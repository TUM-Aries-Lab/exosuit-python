"""Test the main program."""

import time

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


def _mock_exosuit() -> Exosuit:
    """Build an exosuit on mock devices.

    :return: A constructed exosuit.
    :rtype: Exosuit
    """
    return Exosuit(
        ExosuitConfig(
            frequency=DEFAULT_EXOSUIT_FREQUENCY_HZ,
            mock_devices=True,
            imu_cfg=IMUConfig(),
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
