"""Test the main program."""

import math
import time

from exosuit_python.csv_writer import RecordDataColumnNames, SensorSignal
from exosuit_python.definitions import (
    DEFAULT_EXOSUIT_FREQUENCY_HZ,
    EXOSUIT_STANDBY_INTERVAL,
    MODE_SWITCH_1,
    MODE_SWITCH_2,
    MODE_SWITCH_LOGIC,
    OPERATION_SWITCH,
    SWITCH_EVENT_HANDLER_INTERVAL,
    TENSION_SWITCH,
    IMUConfig,
    TensionConfig,
)
from exosuit_python.exosuit import Exosuit, ExosuitConfig, ExosuitStates
from exosuit_python.gpio import MockGPIO


def test_exosuit_initialization():
    """Test if exosuit is initialized with the set config."""
    imu_config = IMUConfig()
    exosuit_config = ExosuitConfig(
        frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ, mock_devices=True, imu_cfg=imu_config
    )
    exosuit = Exosuit(exosuit_config)

    assert exosuit.config == exosuit_config
    exosuit._cleanup()


def test_exosuit_switches():
    """Test if exosuit's state changes correctly upon switch triggers."""
    imu_config = IMUConfig()
    exosuit_config = ExosuitConfig(
        frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ, mock_devices=True, imu_cfg=imu_config
    )
    exosuit = Exosuit(exosuit_config)

    # wait for initialization
    time.sleep(1)
    assert isinstance(exosuit.gpio, MockGPIO)
    assert exosuit._status == ExosuitStates.STANDBY
    # time for the event handler to register changes
    state_wait = EXOSUIT_STANDBY_INTERVAL + SWITCH_EVENT_HANDLER_INTERVAL + 0.1
    # simulate tension switch ON
    exosuit.gpio.simulate_switch(TENSION_SWITCH, exosuit.on_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.PRETENSIONING
    # simulate tension switch OFF
    exosuit.gpio.simulate_switch(TENSION_SWITCH, exosuit.off_signal)
    time.sleep(state_wait + TensionConfig.tensioning_timeout)
    assert exosuit._status == ExosuitStates.STANDBY
    # simulate operation switch ON
    exosuit.gpio.simulate_switch(OPERATION_SWITCH, exosuit.on_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.RUNNING
    # simulate operation switch OFF
    exosuit.gpio.simulate_switch(OPERATION_SWITCH, exosuit.off_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.STANDBY

    # stop exosuit
    exosuit._cleanup()
    assert exosuit._status == ExosuitStates.STOPPED


def test_exosuit_inclination_mode_switch():
    """Test if exosuit's mode changes correctly upon switch triggers."""
    imu_config = IMUConfig()
    exosuit_config = ExosuitConfig(
        frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ, mock_devices=True, imu_cfg=imu_config
    )
    exosuit = Exosuit(exosuit_config)

    # wait for initialization
    time.sleep(1)
    assert isinstance(exosuit.gpio, MockGPIO)

    # test each mode
    for mode, state in MODE_SWITCH_LOGIC.items():
        switch_1 = getattr(exosuit.gpio, state.switch_1)
        switch_2 = getattr(exosuit.gpio, state.switch_2)
        exosuit.gpio.simulate_switch(MODE_SWITCH_1, switch_1)
        exosuit.gpio.simulate_switch(MODE_SWITCH_2, switch_2)
        time.sleep(SWITCH_EVENT_HANDLER_INTERVAL + 0.1)
        assert exosuit.inclination_mode == mode

    exosuit._cleanup()


def _mock_exosuit(record: bool = True) -> Exosuit:
    """Build an exosuit on mock devices.

    :param bool record: Leave recording on. Tests that inspect rows must pass
        False: constructing an Exosuit starts its loop thread, which sits in
        the standby lap appending a row every tick and would otherwise race
        with the rows the test appends itself.
    :return: A constructed exosuit.
    :rtype: Exosuit
    """
    return Exosuit(
        ExosuitConfig(
            frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ,
            mock_devices=True,
            imu_cfg=IMUConfig(),
            record=record,
        )
    )


def _is_collecting(exosuit: Exosuit) -> tuple[bool, bool]:
    """Report whether each limb has an open baseline window.

    :param Exosuit exosuit: Instance under test.
    :return: ``(left, right)`` window states.
    :rtype: tuple[bool, bool]
    """
    return (
        exosuit.controller_left.pre_processor._baseline_removal.is_collecting,
        exosuit.controller_right.pre_processor._baseline_removal.is_collecting,
    )


def test_baseline_removal_trigger_reaches_both_limbs():
    """A one-legged trigger would zero one hip against a stale offset."""
    exosuit = _mock_exosuit()

    exosuit._set_baseline_removal_trigger(True)
    assert _is_collecting(exosuit) == (True, True)

    exosuit._set_baseline_removal_trigger(False)
    assert _is_collecting(exosuit) == (False, False)

    exosuit._cleanup()


def test_a_second_session_opens_a_new_baseline_window():
    """The falling edge at session end is what lets the next session calibrate.

    `_control()` stops being called the moment the status leaves RUNNING, so the
    loop releases the trigger itself. Without that release the trigger stays
    high inside the controller, the next rising edge is not an edge at all, and
    baseline removal works on the first run of the process and silently never
    again.
    """
    exosuit = _mock_exosuit()

    # First session: operator enables, then disables.
    exosuit._set_baseline_removal_trigger(True)
    exosuit._set_baseline_removal_trigger(False)

    # Second session.
    exosuit._set_baseline_removal_trigger(True)

    assert _is_collecting(exosuit) == (True, True)

    exosuit._cleanup()


def test_repeating_the_trigger_does_not_reopen_the_window():
    """The control loop passes the switch state every iteration, not on edges."""
    exosuit = _mock_exosuit()

    exosuit._set_baseline_removal_trigger(True)
    for _ in range(5):
        exosuit._set_baseline_removal_trigger(True)

    left, right = exosuit.controller_left, exosuit.controller_right
    assert left.pre_processor._baseline_removal.is_collecting
    assert right.pre_processor._baseline_removal.is_collecting
    assert left.baseline_offset_rad == 0.0
    assert right.baseline_offset_rad == 0.0

    exosuit._cleanup()


def test_recording_is_on_by_default_and_can_be_switched_off():
    """An unrecorded trial cannot be analysed afterwards, so it defaults on."""
    assert ExosuitConfig(frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ).record is True
    assert (
        ExosuitConfig(frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ, record=False).record
        is False
    )


def test_saving_without_any_rows_writes_nothing():
    """A process that never recorded must not leave an empty file behind."""
    exosuit = _mock_exosuit()

    exosuit._save_recording()

    assert exosuit.csv_writer.rows == []
    exosuit._cleanup()


def test_idle_rows_mark_the_switch_low_and_leave_the_computed_columns_empty():
    """Standby rows carry sensor data only.

    The recording covers the whole process, so the operation_switch column
    varies and the gaps between sessions are visible. The controller is not
    stepped while idle, so its columns say NaN rather than repeating a stale
    value that would look like live output.
    """
    exosuit = _mock_exosuit(record=False)
    raw = SensorSignal(timestamp=1.0, angle_rad=0.4, velocity_rad_per_sec=0.1)
    not_computed = SensorSignal(
        timestamp=1.0, angle_rad=math.nan, velocity_rad_per_sec=math.nan
    )

    exosuit._record_sample(
        raw=(raw, raw),
        filtered=(not_computed, not_computed),
        commands=(math.nan, math.nan),
    )
    row = exosuit.csv_writer.rows[0]

    assert row[RecordDataColumnNames.OPERATION_SWITCH.value] == 0.0
    assert row[RecordDataColumnNames.RAW_ANGLE_LEFT.value] == 0.4
    assert math.isnan(row[RecordDataColumnNames.FILTERED_ANGLE_LEFT.value])
    assert math.isnan(row[RecordDataColumnNames.MOTOR_COMMAND_LEFT.value])

    exosuit._cleanup()


def test_the_recording_accumulates_across_sessions():
    """One run of the process is one file, not one file per switch flick."""
    exosuit = _mock_exosuit(record=False)
    raw = SensorSignal(timestamp=1.0, angle_rad=0.4, velocity_rad_per_sec=0.1)

    for _ in range(3):
        exosuit._record_sample(raw=(raw, raw), filtered=(raw, raw), commands=(0.0, 0.0))

    assert len(exosuit.csv_writer.rows) == 3
    exosuit._cleanup()


def test_both_switches_are_recorded():
    """Row spacing is uniform, so the state cannot be read off the timing.

    Standby and pretensioning both have the operation switch off, so that
    column alone cannot separate them. Recording both switches does: neither
    is standby, tension alone is pretensioning, operation is running.
    """
    exosuit = _mock_exosuit(record=False)
    raw = SensorSignal(timestamp=1.0, angle_rad=0.4, velocity_rad_per_sec=0.1)

    exosuit._tension_switch = True
    exosuit._record_sample(
        raw=(raw, raw), filtered=(raw, raw), commands=(math.nan, math.nan)
    )
    exosuit._tension_switch = False
    exosuit._operation_switch = True
    exosuit._record_sample(raw=(raw, raw), filtered=(raw, raw), commands=(0.0, 0.0))

    rows = exosuit.csv_writer.rows
    tension = RecordDataColumnNames.TENSION_SWITCH.value
    operation = RecordDataColumnNames.OPERATION_SWITCH.value

    assert (rows[0][tension], rows[0][operation]) == (1.0, 0.0)
    assert (rows[1][tension], rows[1][operation]) == (0.0, 1.0)

    exosuit._cleanup()


def test_a_switch_edge_signals_the_handler():
    """The callbacks signal the handler instead of letting it poll for the flag.

    The handler is stopped first, deliberately. The event is a one-shot the
    handler consumes -- it wakes, clears, and carries on -- so asserting on it
    while that thread is live tests which of the two ran first, not whether the
    callback signalled. An earlier version of this test did exactly that and
    failed on all three Python versions.

    The latency this buys is not asserted here: it depends on MockGPIO's own
    0.2 s edge-detection poll and on CI runner load. See the PR for the
    measurement.
    """
    exosuit = _mock_exosuit(record=False)
    exosuit._cleanup()  # stops the handler, so nothing consumes the event

    exosuit._switch_event.clear()
    exosuit._operation_callback(OPERATION_SWITCH)
    assert exosuit._switch_event.is_set()

    exosuit._switch_event.clear()
    exosuit._tension_callback(TENSION_SWITCH)
    assert exosuit._switch_event.is_set()
