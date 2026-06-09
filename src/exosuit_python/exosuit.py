"""Exosuit configuration."""

import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum

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
    POWER_SWITCH,
    SWITCH_EVENT_HANDLER_INTERVAL,
    SWITCH_OFF,
    SWITCH_ON,
    TENSION_SWITCH,
    THREAD_JOIN_TIMEOUT,
    IMUConfig,
    TensionConfig,
)
from exosuit_python.gpio import MockGPIO
from exosuit_python.motor import MockMotor
from exosuit_python.utils import convert_rad_per_sec_to_rpm


@dataclass
class ExosuitConfig:
    """Exosuit configuration.

    Attributes:
        frequency: Exosuit frequency in Hz.
        mock: flag to use mock devices.
        imu_cfg: IMU config that defines the IMU to use for each leg.

    """

    frequency: float
    mock: bool = False
    imu_cfg: IMUConfig = field(default_factory=IMUConfig)


class ExosuitStates(IntEnum):
    """Enum for exosuit states."""

    INITIALIZING = 0
    STANDBY = 1
    PRETENSIONING = 2
    RUNNING = 3
    STOPPED = 4


class Exosuit:
    """Tendon-based soft exoskeleton."""

    def __init__(self, config: ExosuitConfig) -> None:
        """Initialize the exosuit.

        :param config: Exosuit configuration
        """
        self.config: ExosuitConfig = config
        self._status: ExosuitStates = ExosuitStates.INITIALIZING

        # GPIO for switches
        if self.config.mock or GPIO is None:
            self.gpio = MockGPIO()
        else:
            self.gpio = GPIO

        if (
            hasattr(self.gpio, SWITCH_ON)
            and hasattr(self.gpio, SWITCH_OFF)
            and hasattr(self.gpio, BOTH)
        ):
            self.on_signal = getattr(self.gpio, SWITCH_ON)
            self.off_signal = getattr(self.gpio, SWITCH_OFF)
            self.both_signal = getattr(self.gpio, BOTH)
        else:
            logger.error(
                f"GPIO signal attribute '{SWITCH_ON}', '{SWITCH_OFF}', or '{BOTH}' not found."
            )
            return

        self._power_switch: bool = False
        self._tension_switch: bool = False

        # main loop thread and switch handler thread
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        self.switch_thread: threading.Thread = threading.Thread(
            target=self._switch_event_handler, daemon=True
        )

        self.imu_left: IMUManager
        self.imu_right: IMUManager

        self.controller_left = WalkOnController(reverse=False)
        self.controller_right = WalkOnController(reverse=True)

        self.motor_left: CubeMarsAK606v3 | MockMotor
        self.motor_right: CubeMarsAK606v3 | MockMotor

        if self.config.mock:
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
            self.gpio.setup(POWER_SWITCH, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)
            self.gpio.setup(TENSION_SWITCH, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)

            self.gpio.add_event_detect(
                POWER_SWITCH,
                self.both_signal,
                callback=self._power_callback,
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
                if self._power_switch and not self._tension_switch:
                    logger.info("State change: standby -> running")
                    self._status = ExosuitStates.RUNNING
                elif self._tension_switch and not self._power_switch:
                    logger.info("State change: standby -> pretensioning")
                    self._status = ExosuitStates.PRETENSIONING
            elif self._status == ExosuitStates.RUNNING:
                if not self._power_switch:
                    logger.info("State change: running -> standby")
                    self._status = ExosuitStates.STANDBY
            elif self._status == ExosuitStates.PRETENSIONING:
                pass  # state change handled in _loop()
            time.sleep(SWITCH_EVENT_HANDLER_INTERVAL)

    def _power_callback(self, channel: int) -> None:
        """Handle power switch states triggered by signal events."""
        power_state = self.gpio.input(POWER_SWITCH)
        if power_state == self.on_signal:
            self._power_switch = True
        elif power_state == self.off_signal:
            self._power_switch = False
        else:
            logger.warning(f"Unrecognized power switch state: {power_state}")

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
        self.gpio.cleanup()
        self.imu_left.stop()
        self.imu_right.stop()
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

        if data_right is None or data_left is None:
            raise TypeError

        timestamp_right = data_right.timestamp
        signal_right = SensorSignal(
            angle_rad=data_right.quat.to_euler(seq="xyz").z,
            velocity_rad_per_sec=data_right.device_data.gyro.z,
        )
        command_right = self.controller_right.step(
            timestamp=timestamp_right, curr_signal=signal_right
        )

        timestamp_left = data_left.timestamp
        signal_left = SensorSignal(
            angle_rad=data_left.quat.to_euler(seq="xyz").z,
            velocity_rad_per_sec=data_left.device_data.gyro.z,
        )
        command_left = self.controller_left.step(
            timestamp=timestamp_left, curr_signal=signal_left
        )

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
        try:
            left_init: bool = False
            right_init: bool = False
            sensor_managers = IMUFactory.detect_and_create(
                free_threading=True,
                log_data=False,
            )
            detected_imus = len(sensor_managers)
            if detected_imus < 2:
                raise ValueError(f"Wrong number of IMUs detected: {detected_imus} < 2")
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
                raise RuntimeError("No detected IMUs matches left leg config")
            if not right_init:
                raise RuntimeError("No detected IMUs matches right leg config")
            return True
        except Exception as err:
            logger.error(f"Exosuit exception: '{err}'. Check IMU connections.")
            return False

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
