"""Exosuit configuration."""

import threading
import time
import warnings
from dataclasses import dataclass, field

from loguru import logger

try:
    from Jetson import GPIO
except Exception:
    logger.warning("Jetson GPIO import failed. Are you running on the Jetson?")
    GPIO = None
from hip_controller.control.app import WalkOnController
from hip_controller.definitions import SensorSignal
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from motor_python.cube_mars_motor import CubeMarsAK606v3

from exosuit_python.definitions import (
    BOTH,
    EXOSUIT_STANDBY_INTERVAL,
    GPIO_SWITCH_BOUNCETIME,
    MODE_SWITCH_1,
    MODE_SWITCH_2,
    MODE_SWITCH_LOGIC,
    OPERATION_SWITCH,
    SWITCH_EVENT_HANDLER_INTERVAL,
    TENSION_SWITCH,
    THREAD_JOIN_TIMEOUT,
    ExosuitStates,
    IMUConfig,
    InclinationModes,
    SwitchStates,
    TensionConfig,
    controller_modes,
)
from exosuit_python.gpio import MockGPIO
from exosuit_python.motor import MockMotor
from exosuit_python.utils import convert_rad_per_sec_to_rpm


@dataclass
class ExosuitConfig:
    """Exosuit configuration.

    Attributes:
        frequency: Exosuit frequency in Hz.
        mock_devices: flag to use mock devices.
        test_gpio: flag to use Jetson GPIO (for switch testing on the Jetson) and use mock_devices.
        imu_cfg: IMU config that defines the IMU to use for each leg.

    """

    frequency: float
    mock_devices: bool = False
    test_gpio: bool = False
    imu_cfg: IMUConfig = field(default_factory=IMUConfig)


class Exosuit:
    """Tendon-based soft exoskeleton."""

    def __init__(self, config: ExosuitConfig) -> None:
        """Initialize the exosuit.

        :param config: Exosuit configuration
        """
        self.config: ExosuitConfig = config
        self._status: ExosuitStates = ExosuitStates.INITIALIZING

        # GPIO for switches
        if GPIO is None or (not self.config.test_gpio and self.config.mock_devices):
            self.gpio = MockGPIO()
        else:
            self.gpio = GPIO

        if (
            hasattr(self.gpio, SwitchStates.ON)
            and hasattr(self.gpio, SwitchStates.OFF)
            and hasattr(self.gpio, BOTH)
        ):
            self.on_signal = getattr(self.gpio, SwitchStates.ON)
            self.off_signal = getattr(self.gpio, SwitchStates.OFF)
            self.both_signal = getattr(self.gpio, BOTH)
        else:
            logger.error(
                f"GPIO signal attribute '{SwitchStates.ON}', '{SwitchStates.OFF}', or '{BOTH}' not found."
            )
            return

        self._operation_switch: bool = False
        self._tension_switch: bool = False

        # main loop thread and switch handler thread
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        self.switch_thread: threading.Thread = threading.Thread(
            target=self._switch_event_handler, daemon=True
        )
        self.inclination_mode: InclinationModes = InclinationModes.LEVEL_GROUND
        self._prev_inclination_mode: InclinationModes = InclinationModes.LEVEL_GROUND

        self.imu_left: IMUManager
        self.imu_right: IMUManager

        self.controller_left = WalkOnController(reverse=False)
        self.controller_right = WalkOnController(reverse=True)

        self.motor_left: CubeMarsAK606v3 | MockMotor
        self.motor_right: CubeMarsAK606v3 | MockMotor

        if self.config.mock_devices or self.config.test_gpio:
            self.motor_left = MockMotor()
            self.motor_right = MockMotor()
        else:
            self.motor_left = CubeMarsAK606v3()
            self.motor_right = CubeMarsAK606v3()

        # initialization calls
        if not self._initialize_imus():
            logger.error("IMU initialization failed. Exosuit not started.")
            return
        if not self._initialize_motors():
            logger.error("Motor initialization failed. Exosuit not started.")
            return

        if self._initialize_gpio():
            self._start()
        else:
            logger.error("GPIO initialization failed. Exosuit not started.")

    def _initialize_gpio(self) -> bool:
        """Initialize and set up Jetson GPIO switches.

        :return: True if successful, False otherwise
        """
        try:
            self.gpio.setmode(self.gpio.BOARD)
            self.gpio.setup(OPERATION_SWITCH, self.gpio.IN)
            self.gpio.setup(TENSION_SWITCH, self.gpio.IN)
            self.gpio.setup(MODE_SWITCH_1, self.gpio.IN)
            self.gpio.setup(MODE_SWITCH_2, self.gpio.IN)

            self.gpio.add_event_detect(
                OPERATION_SWITCH,
                self.both_signal,
                callback=self._operation_callback,
                bouncetime=GPIO_SWITCH_BOUNCETIME,
            )

            self.gpio.add_event_detect(
                TENSION_SWITCH,
                self.both_signal,
                callback=self._tension_callback,
                bouncetime=GPIO_SWITCH_BOUNCETIME,
            )

            return True
        except Exception as err:
            logger.error(f"GPIO init failure: {err}")
            self.gpio.cleanup()
            return False

    def _switch_event_handler(
        self,
    ) -> None:  # TODO: implement proper state machine if needed
        """Monitor switch states and handle exosuit status changes.

        :return: None
        """
        while self._status != ExosuitStates.STOPPED:
            if self._status == ExosuitStates.STANDBY:
                if self._operation_switch and not self._tension_switch:
                    logger.info("State change: standby -> running")
                    self._status = ExosuitStates.RUNNING
                elif self._tension_switch and not self._operation_switch:
                    logger.info("State change: standby -> pretensioning")
                    self._status = ExosuitStates.PRETENSIONING
            elif self._status == ExosuitStates.RUNNING:
                if not self._operation_switch:
                    logger.info("State change: running -> standby")
                    self._status = ExosuitStates.STANDBY
            elif self._status == ExosuitStates.PRETENSIONING:
                pass  # state change handled in _loop()

            switch_1 = (
                SwitchStates.ON
                if self.gpio.input(MODE_SWITCH_1) == self.on_signal
                else SwitchStates.OFF
            )
            switch_2 = (
                SwitchStates.ON
                if self.gpio.input(MODE_SWITCH_2) == self.on_signal
                else SwitchStates.OFF
            )
            mode = self._get_mode(switch_1=switch_1, switch_2=switch_2)
            if mode is not None:
                self.inclination_mode = mode

            time.sleep(SWITCH_EVENT_HANDLER_INTERVAL)

    def _operation_callback(self, channel: int) -> None:
        """Handle operation switch states triggered by signal events."""
        opration_state = self.gpio.input(OPERATION_SWITCH)
        if opration_state == self.on_signal:
            self._operation_switch = True
        elif opration_state == self.off_signal:
            self._operation_switch = False
        else:
            logger.warning(f"Unrecognized operation switch state: {opration_state}")

    def _tension_callback(self, channel: int) -> None:
        """Handle tension switch states triggered by signal events."""
        tension_state = self.gpio.input(TENSION_SWITCH)
        if tension_state == self.on_signal:
            self._tension_switch = True
        elif tension_state == self.off_signal:
            self._tension_switch = False
        else:
            logger.warning(f"Unrecognized tension switch state: {tension_state}")

    def _start(self) -> None:
        """Start the IMUs and Motors.

        :return: None
        """
        logger.info(f"Starting Exosuit at '{self.config.frequency}' Hz.")
        try:
            logger.debug("Starting IMUs")
            self.imu_left.start()
            self.imu_right.start()

            logger.debug("Starting Motors")
            logger.debug("Starting Controller")

            # TODO: get mode
            self._status = ExosuitStates.STANDBY
            logger.info("Exosuit status: standby")
            # Start main control loop
            self.thread.start()
            self.switch_thread.start()

        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")
            self._cleanup()

    def _cleanup(self) -> None:
        """Clean up the soft exoskeleton.

        :return: None
        """
        logger.info("Cleaning up exosuit.")
        self._status = ExosuitStates.STOPPED
        with warnings.catch_warnings():
            # suppress warning from GPIO when no channels has been set up
            warnings.simplefilter("ignore", RuntimeWarning)
            self.gpio.cleanup()
        try:  # imu attributes can be unassigned in case of failure
            self.imu_left.stop()
            self.imu_right.stop()
        except AttributeError:
            pass
        self.motor_left.close()
        self.motor_right.close()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        if self.switch_thread is not None and self.switch_thread.is_alive():
            self.switch_thread.join(timeout=THREAD_JOIN_TIMEOUT)
        logger.success("Exosuit shutdown.")

    def _loop(self) -> None:
        """Run main control loop.

        :return: None
        """
        while self._status != ExosuitStates.STOPPED:
            while self._status == ExosuitStates.RUNNING:
                try:
                    self._control()
                except TypeError as err:
                    logger.error(f"Failed getting data from the IMU: '{err}'.")
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

                time.sleep(1 / self.config.frequency)

            while self._status == ExosuitStates.PRETENSIONING:
                try:
                    self._pretension()
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

                time.sleep(TensionConfig.torque_check_interval)

            time.sleep(EXOSUIT_STANDBY_INTERVAL)

    def _control(self) -> None:
        """Execute one iteration of control loop."""
        data_right = self.imu_right.get_data()
        data_left = self.imu_left.get_data()

        if self._prev_inclination_mode != self.inclination_mode:
            logger.info(
                f"Mode change:{self._prev_inclination_mode.name} -> {self.inclination_mode.name}"
            )
            self.controller_left.amplitude_modulation.set_mode(
                controller_modes[self.inclination_mode]
            )
            self._prev_inclination_mode = self.inclination_mode

        if data_right is None or data_left is None:
            raise TypeError

        timestamp_right = data_right.timestamp
        signal_right = SensorSignal(
            angle_rad=data_right.quat.to_euler(seq="xyz").z,
            velocity_rad_per_sec=data_right.device_data.gyro.z,
            timestamp=timestamp_right,
        )
        command_right = self.controller_right.step(curr_signal=signal_right)

        timestamp_left = data_left.timestamp
        signal_left = SensorSignal(
            angle_rad=data_left.quat.to_euler(seq="xyz").z,
            velocity_rad_per_sec=data_left.device_data.gyro.z,
            timestamp=timestamp_left,
        )
        command_left = self.controller_left.step(curr_signal=signal_left)

        self.motor_left.set_velocity(convert_rad_per_sec_to_rpm(command_left))

        self.motor_right.set_velocity(convert_rad_per_sec_to_rpm(command_right))

    def _pretension(self) -> None:
        """Execute one iteration of pretensioning loop."""
        left_motor_torque = 0.85  # TODO place holder, get actual torque here
        right_motor_torque = 0.85  # TODO place holder, get actual torque here

        # Only apply velocity if motor hasn't reached threshold
        if left_motor_torque < TensionConfig.motor_torque_limit:
            self.motor_left.set_velocity(TensionConfig.tensioning_velocity)
        else:
            self.motor_left.set_velocity(0)

        if right_motor_torque < TensionConfig.motor_torque_limit:
            self.motor_right.set_velocity(TensionConfig.tensioning_velocity)
        else:
            self.motor_right.set_velocity(0)

        # Exit pretensioning when both motors reach threshold and switch is released
        if (
            left_motor_torque >= TensionConfig.motor_torque_limit
            and right_motor_torque >= TensionConfig.motor_torque_limit
            and not self._tension_switch
        ):
            time.sleep(TensionConfig.tensioning_timeout)
            logger.info("State change: pretensioning -> standby")
            self._status = ExosuitStates.STANDBY

    def _initialize_imus(self) -> bool:
        """Initialize IMUs.

        :return: True if successful, False otherwise
        """
        left_init: bool = False
        right_init: bool = False

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            sensor_managers = IMUFactory.detect_and_create(
                free_threading=True,
                log_data=False,
                create_mock=self.config.mock_devices or self.config.test_gpio,
            )
        detected_imus = len(sensor_managers)
        if detected_imus < 2:
            logger.error(f"Wrong number of IMUs detected: {detected_imus} < 2")
        for idx in range(
            detected_imus
        ):  # Match each leg with the IMU according to IMUConfig
            manager = sensor_managers[idx]
            if (
                not left_init
                and manager.i2c_id == self.config.imu_cfg.left_leg_bus
                and manager.imu_descriptor == self.config.imu_cfg.left_leg_descr
            ):
                self.imu_left = manager
                left_init = True
                continue
            if (
                manager.i2c_id == self.config.imu_cfg.right_leg_bus
                and manager.imu_descriptor == self.config.imu_cfg.right_leg_descr
            ):
                self.imu_right = manager
                right_init = True
        if not left_init:
            logger.error("No detected IMUs matches left leg config")
        if not right_init:
            logger.error("No detected IMUs matches right leg config")
        return left_init and right_init

    def _initialize_motors(self) -> bool:
        """Initialize Motors.

        :return: True if successful, False otherwise
        """
        try:
            communication_status = True
            if not self.motor_left.check_communication():
                logger.error("Left Motor not responding. Check power and connections.")
                communication_status = False
            if not self.motor_right.check_communication():
                logger.error("Right Motor not responding. Check power and connections.")
                communication_status = False
            return communication_status
        except Exception as err:
            logger.error(f"Exosuit exception: '{err}'. Check Motor connections.")
            return False

    def _get_mode(
        self, switch_1: SwitchStates, switch_2: SwitchStates
    ) -> InclinationModes | None:
        """Get inclination mode based on the current switch states. Return None if unrecognized.

        :param switch_1: state of the first channel of the switch.
        :param switch_2: state of the second channel of the switch.
        :return: the matched inclination mode, or None if unrecognized.
        """
        for mode, state in MODE_SWITCH_LOGIC.items():
            if switch_1 == state.switch_1 and switch_2 == state.switch_2:
                return mode

        return None
