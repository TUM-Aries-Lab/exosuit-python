"""Test the main program."""

import math
import time
from types import SimpleNamespace

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
    MotorCommandConfig,
    MotorSaturation,
    TensionConfig,
)
from exosuit_python.exosuit import Exosuit, ExosuitConfig, ExosuitStates
from exosuit_python.gpio import MockGPIO
from exosuit_python.motor import MockMotor


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
    time.sleep(state_wait + TensionConfig.stop_hold_time)
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


def _mock_gpio(exosuit: Exosuit) -> MockGPIO:
    """Return the GPIO, narrowed to the mock these tests build.

    simulate_switch() exists only on the mock, so the union has to be narrowed
    before it can be driven. The assert doubles as a check that the fixture
    really did build a mock rather than reaching for the Jetson's pins.
    """
    gpio = exosuit.gpio
    assert isinstance(gpio, MockGPIO)
    return gpio


def _mock_motor(exosuit: Exosuit) -> MockMotor:
    """Return the left motor, narrowed to the mock these tests build.

    Exosuit types its motors as ``CubeMarsAK806v2CAN | MockMotor``, and the
    recorded command only exists on the mock, so the union has to be narrowed
    before it can be read. The assert doubles as a check that the fixture
    really did build a mock rather than reaching for CAN hardware.
    """
    motor = exosuit.motor_left
    assert isinstance(motor, MockMotor)
    return motor


def test_velocity_commands_go_out_in_rad_per_sec_unscaled():
    """The assist command must reach the motor as rad/s, at full scale.

    This pins the fix for a silent 126x error. The command used to be pushed
    through convert_rad_per_sec_to_rpm(), which produced mechanical RPM for a
    parameter documented as electrical RPM; the motor then divided by pole
    pairs times gear ratio (21 * 6). Both quantities are plausible ints, so
    nothing failed loudly -- the suit just assisted at a 126th of the command.
    """
    exosuit = _mock_exosuit(record=False)

    motor = _mock_motor(exosuit)

    exosuit._command_velocity(motor, 2.5)

    command = motor.last_mit_command
    assert command is not None
    assert command["vel_rad_s"] == 2.5
    assert command["kp"] == MotorCommandConfig.kp
    assert command["kd"] == MotorCommandConfig.velocity_kd
    assert command["torque_ff_nm"] == MotorCommandConfig.torque_ff_nm

    exosuit._cleanup()


def test_a_zero_velocity_command_does_not_tear_down_mit_mode():
    """Zero assist must stay a command, not a stop.

    ``set_velocity`` maps zero to ``stop()``, and on the CAN class ``stop()``
    disables MIT mode. The assist command crosses zero every stride, so going
    through that path would tear MIT mode down and rebuild it continuously.
    """
    exosuit = _mock_exosuit(record=False)

    motor = _mock_motor(exosuit)

    exosuit._command_velocity(motor, 0.0)

    command = motor.last_mit_command
    assert command is not None
    assert command["vel_rad_s"] == 0.0

    exosuit._cleanup()


def test_pretensioning_uses_its_own_damping_gain():
    """The two regimes keep independent Kd knobs."""
    exosuit = _mock_exosuit(record=False)

    motor = _mock_motor(exosuit)

    exosuit._command_velocity(motor, 3.0, velocity_kd=TensionConfig.mit_velocity_kd)

    command = motor.last_mit_command
    assert command is not None
    assert command["kd"] == TensionConfig.mit_velocity_kd

    exosuit._cleanup()


def test_switching_off_a_running_session_releases_the_motors():
    """Switching off must reach the motors, not just the state machine.

    set_mit_mode() installs a keep-alive thread that re-transmits the last MIT
    payload until stop() or close(). Leaving RUNNING without stopping therefore
    leaves both motors driving at the last assist command with the operator's
    switch already off. The old UART path had no keep-alive, so this only
    became reachable with the move to CAN.
    """
    exosuit = _mock_exosuit(record=False)
    time.sleep(1)
    state_wait = EXOSUIT_STANDBY_INTERVAL + SWITCH_EVENT_HANDLER_INTERVAL + 0.1

    gpio = _mock_gpio(exosuit)
    gpio.simulate_switch(OPERATION_SWITCH, exosuit.on_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.RUNNING

    motor = _mock_motor(exosuit)
    motor.stop_calls = 0

    gpio.simulate_switch(OPERATION_SWITCH, exosuit.off_signal)
    time.sleep(state_wait)

    assert exosuit._status == ExosuitStates.STANDBY
    assert motor.stop_calls >= 1

    exosuit._cleanup()


def test_pretensioning_does_not_pull_again_while_the_switch_is_held():
    """Reaching the threshold ends the pull; it must not re-arm.

    DISABLE is terminal on the rig. Resetting the charts and returning to
    standby while the switch is still held lets the switch handler send the
    loop straight back into PRETENSIONING, winding the tendon tighter on every
    cycle for as long as the wearer holds the switch.

    Polling is checked too: on the real motor the status request frame is
    byte-identical to the MIT enable frame, so continuing to read torque from
    a stopped motor would re-energise it.
    """
    exosuit = _mock_exosuit(record=False)
    time.sleep(1)
    state_wait = EXOSUIT_STANDBY_INTERVAL + SWITCH_EVENT_HANDLER_INTERVAL + 0.1

    gpio = _mock_gpio(exosuit)
    gpio.simulate_switch(TENSION_SWITCH, exosuit.on_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.PRETENSIONING

    # Long enough for the torque to cross the threshold and the STOP hold to
    # run out, so both charts are terminal.
    time.sleep(TensionConfig.stop_hold_time + 1.5)
    motor = _mock_motor(exosuit)
    assert exosuit._status == ExosuitStates.PRETENSIONING

    settled_pulls = motor.pull_commands
    settled_polls = motor.get_current_calls
    time.sleep(1.0)

    assert motor.pull_commands == settled_pulls
    assert motor.get_current_calls == settled_polls

    # And releasing the switch is what ends it.
    gpio.simulate_switch(TENSION_SWITCH, exosuit.off_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.STANDBY

    exosuit._cleanup()


def test_releasing_the_switch_leaves_pretensioning_even_if_it_never_armed():
    """A tap too short to arm the charts must not strand the loop.

    RESET has no exit for on_off == 0, so charts that never reached TENSIONING
    never report finished. Keying the exit on the switch rather than on the
    charts means a tap the control thread missed cannot freeze the loop in
    PRETENSIONING -- which would also leave the operation switch dead, since
    the handler's PRETENSIONING branch does nothing.
    """
    exosuit = _mock_exosuit(record=False)
    exosuit._tension_switch = False
    exosuit._status = ExosuitStates.PRETENSIONING

    exosuit._pretension()

    assert exosuit._status == ExosuitStates.STANDBY
    assert _mock_motor(exosuit).stop_calls >= 1

    exosuit._cleanup()


def test_an_excessive_velocity_command_is_saturated():
    """The assist command is bounded before it reaches the motor.

    The rig saturates inside pack_mit_command(); this repo had no equivalent,
    and once the command stopped being divided by 126 the only remaining limit
    was the motor profile's own +/-76 rad/s.
    """
    exosuit = _mock_exosuit(record=False)
    motor = _mock_motor(exosuit)
    limit = MotorSaturation.velocity_rad_per_sec[1]

    exosuit._command_velocity(motor, limit + 30.0)
    command = motor.last_mit_command
    assert command is not None
    assert command["vel_rad_s"] == limit

    exosuit._command_velocity(motor, -limit - 30.0)
    command = motor.last_mit_command
    assert command is not None
    assert command["vel_rad_s"] == -limit

    exosuit._cleanup()


def test_ordinary_commands_pass_through_untouched():
    """Saturation must not quietly reshape commands that were already legal."""
    exosuit = _mock_exosuit(record=False)
    motor = _mock_motor(exosuit)

    for velocity in (
        0.0,
        2.5,
        -2.5,
        TensionConfig.tensioning_velocity_left_rad_per_sec,
    ):
        exosuit._command_velocity(motor, velocity)
        command = motor.last_mit_command
        assert command is not None
        assert command["vel_rad_s"] == velocity

    exosuit._cleanup()


def test_the_damping_gain_is_bounded_too():
    """Kd is packed against a 0..5 range, so an out-of-range value is clamped."""
    exosuit = _mock_exosuit(record=False)
    motor = _mock_motor(exosuit)

    exosuit._command_velocity(motor, 2.0, velocity_kd=99.0)

    command = motor.last_mit_command
    assert command is not None
    assert command["kd"] == MotorSaturation.kd[1]

    exosuit._cleanup()


def test_neither_limb_is_reversed_on_this_suit():
    """hip-controller mirrors the right limb by default; this suit must not.

    That default exists for rigs whose two motors are mounted opposite each
    other. Bench recordings on 2026-09-18 showed this suit is not such a rig:
    a positive command wound the cable in on both motors, and flexion raised
    the angle and the velocity on both legs alike. With nothing inverted the
    flag has nothing to cancel, and leaving it set inverts the right leg's
    assist on its own -- silently, since an inverted assist still produces a
    plausible-looking command.
    """
    exosuit = _mock_exosuit(record=False)

    assert exosuit.controller_config.left_limb_reverse is False
    assert exosuit.controller_config.right_limb_reverse is False

    exosuit._cleanup()


def test_fresh_feedback_is_read_without_sending_a_request():
    """Reading torque must not poke the motor while it is being driven.

    get_current() sends a status request and blocks on the reply, and that
    request frame is byte-identical to the MIT enable frame -- so polling
    twice a tick interleaves enter-motor-mode frames with the keep-alive
    thread's velocity commands. Every MIT command already draws a feedback
    frame, so the value is in hand and costs nothing to read.
    """
    exosuit = _mock_exosuit(record=False)
    motor = _mock_motor(exosuit)
    # The cache a real CAN motor keeps, which the mock has no reason to.
    motor._last_feedback = SimpleNamespace(current_amps=0.42)
    motor._last_feedback_monotonic = time.monotonic()
    before = motor.get_current_calls

    assert exosuit._read_torque(motor) == 0.42
    assert motor.get_current_calls == before

    exosuit._cleanup()


def test_stale_feedback_falls_back_to_a_real_request():
    """A motor that has stopped answering is exactly when a request is worth it."""
    exosuit = _mock_exosuit(record=False)
    motor = _mock_motor(exosuit)
    motor._last_feedback = SimpleNamespace(current_amps=0.42)
    motor._last_feedback_monotonic = time.monotonic() - 10.0
    before = motor.get_current_calls

    exosuit._read_torque(motor)

    assert motor.get_current_calls == before + 1

    exosuit._cleanup()


def test_a_motor_with_no_cache_still_reads():
    """Nothing may depend on a transport detail the mock does not have."""
    exosuit = _mock_exosuit(record=False)
    motor = _mock_motor(exosuit)

    assert exosuit._read_torque(motor) is not None

    exosuit._cleanup()


def test_a_late_tick_cannot_rail_the_filter():
    """Real elapsed time, but capped.

    The loop does not keep its nominal period -- a bench run measured a worst
    case of 229 ms against a 10 ms target -- so the chart is stepped with the
    time that actually passed. Uncapped, a stall would hand a 25 rad/s filter
    a step of hundreds of milliseconds and rail it.
    """
    exosuit = _mock_exosuit(record=False)
    nominal = 1 / exosuit.config.frequency

    exosuit._last_pretension_tick = None
    assert exosuit._elapsed_since_last_tick() == nominal

    exosuit._last_pretension_tick = time.monotonic() - 5.0
    capped = exosuit._elapsed_since_last_tick()

    assert capped == TensionConfig.max_time_step_periods * nominal
    assert capped < 5.0

    exosuit._cleanup()


def test_the_loop_paces_on_a_schedule_not_a_fixed_delay():
    """Sleeping a whole period after the work makes the rate unreachable.

    A bench run measured 78.6 Hz against 100 Hz configured, the period being
    the sleep plus however long the work took. That is not cosmetic:
    BasicConfig is handed the configured value and hip-controller derives its
    notches and baseline window from it, so a loop running a fifth slower than
    it claims mistunes the whole filter chain.
    """
    exosuit = _mock_exosuit(record=False)
    period = 1 / exosuit.config.frequency

    # A tick whose work took most of the period should sleep only the rest.
    due = time.monotonic() - period * 0.8
    started = time.monotonic()
    next_due = exosuit._sleep_until_due(due)
    slept = time.monotonic() - started

    assert slept < period * 0.5
    assert next_due == due + period

    exosuit._cleanup()


def test_a_late_tick_restarts_the_schedule_instead_of_catching_up():
    """Bursting back-to-back iterations is the wrong answer to falling behind."""
    exosuit = _mock_exosuit(record=False)
    period = 1 / exosuit.config.frequency

    overdue = time.monotonic() - period * 50
    started = time.monotonic()
    next_due = exosuit._sleep_until_due(overdue)

    assert time.monotonic() - started < period  # did not sleep
    assert next_due >= started  # rebased on now, not on the missed schedule
    assert next_due < started + period

    exosuit._cleanup()
