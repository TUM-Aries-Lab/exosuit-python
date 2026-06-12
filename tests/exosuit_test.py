"""Test the main program."""

import time

from exosuit_python.definitions import (
    DEFAULT_EXOSUIT_FREQUENCY_HZ,
    EXOSUIT_STANDBY_INTERVAL,
    MODE_SWITCH_1,
    MODE_SWITCH_2,
    MODE_SWITCH_LOGIC,
    POWER_SWITCH,
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
    # simulate power switch ON
    exosuit.gpio.simulate_switch(POWER_SWITCH, exosuit.on_signal)
    time.sleep(state_wait)
    assert exosuit._status == ExosuitStates.RUNNING
    # simulate power switch OFF
    exosuit.gpio.simulate_switch(POWER_SWITCH, exosuit.off_signal)
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
