"""Exosuit configuration."""

import threading
import time
from dataclasses import dataclass
from enum import IntEnum

from hip_controller.control.app import WalkOnController
from hip_controller.definitions import SensorSignal
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from Jetson import GPIO
from loguru import logger
from motor_python.cube_mars_motor import CubeMarsAK606v3

from exosuit_python.definitions import (
    EXOSUIT_STANDBY_INTERVAL,
    GPIO_SWITCH_BOUNCETIME,
    POWER_SWITCH,
    SWITCH_EVENT_HANDLER_INTERVAL,
    TENSION_SWITCH,
    THREAD_JOIN_TIMEOUT,
    IMUConfig,
    TensionConfig,
)
from exosuit_python.utils import convert_rad_per_sec_to_rpm


@dataclass
class ExosuitConfig:
    """Exosuit configuration."""

    frequency: float
    mock: bool = False


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
        self._power_switch: bool = False
        self._tension_switch: bool = False

        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)

        self.imu_left: IMUManager
        self.imu_right: IMUManager

        self.controller_left = WalkOnController(reverse=False)
        self.controller_right = WalkOnController(reverse=True)

        self.motor_left: CubeMarsAK606v3 = CubeMarsAK606v3()
        self.motor_right = ["Motor right."]  # place holder

        self.imu_initialized: bool = self._initialize_imus()
        self.motors_initialized: bool = self._initialize_motors()

        self._gpio_setup()
        self._start()
        self._switch_event_handler()

    def _gpio_setup(self) -> None:
        """Set up Jetson GPIO switches."""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(POWER_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(TENSION_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(
            POWER_SWITCH,
            GPIO.FALLING,  # TODO: wiring
            callback=self._power_on_callback,
            bouncetime=GPIO_SWITCH_BOUNCETIME,
        )

        GPIO.add_event_detect(
            POWER_SWITCH,
            GPIO.RISING,  # TODO: wiring
            callback=self._power_off_callback,
            bouncetime=GPIO_SWITCH_BOUNCETIME,
        )

        GPIO.add_event_detect(
            TENSION_SWITCH,
            GPIO.FALLING,  # TODO: wiring
            callback=self._tension_on_callback,
            bouncetime=GPIO_SWITCH_BOUNCETIME,
        )

        GPIO.add_event_detect(
            TENSION_SWITCH,
            GPIO.RISING,  # TODO: wiring
            callback=self._tension_off_callback,
            bouncetime=GPIO_SWITCH_BOUNCETIME,
        )

    def _switch_event_handler(self) -> None:
        """Monitor switch states and handle exosuit status changes."""
        try:
            while True:
                if self._status == ExosuitStates.STANDBY:
                    if self._power_switch and not self._tension_switch:
                        logger.info("State change: standby -> running")
                        self._status = ExosuitStates.RUNNING
                    elif self._tension_switch and not self._power_switch:
                        logger.info("State change: standby -> pretensioning")
                        self._status = ExosuitStates.PRETENSIONING
                        self._pretension()
                elif self._status == ExosuitStates.RUNNING:
                    if not self._power_switch:
                        logger.info("State change: running -> standby")
                        self._status = ExosuitStates.STANDBY
                elif self._status == ExosuitStates.PRETENSIONING:
                    if not self._tension_switch:
                        logger.info("State change: pretensioning -> standby")
                        self._status = ExosuitStates.STANDBY
                elif self._status == ExosuitStates.STOPPED:
                    break
                time.sleep(SWITCH_EVENT_HANDLER_INTERVAL)
        finally:
            self._cleanup()

    def _pretension(self) -> None:
        """Pretension the tendons by turning the motors."""
        self.motor_left.set_velocity(TensionConfig.tensioning_velocity)
        # TODO: right motor

    def _power_on_callback(self, channel) -> None:
        self._power_switch = True

    def _power_off_callback(self, channel) -> None:
        self._power_switch = False

    def _tension_on_callback(self, channel) -> None:
        self._tension_switch = True

    def _tension_off_callback(self, channel) -> None:
        self._tension_switch = False

    def _start(self) -> None:
        """Start the IMUs and Motors."""
        if not self.imu_initialized:
            logger.error("IMU initialization failed. Exosuit not started.")
            return
        if not self.motors_initialized:
            logger.error("Motor initialization failed. Exosuit not started.")
            return

        logger.info(f"Starting Exosuit at '{self.config.frequency}' Hz.")
        try:
            logger.debug("Starting IMUs")
            self.imu_left.start()
            self.imu_right.start()

            logger.debug("Starting Motors")
            logger.debug("Starting Controller")

            self._status = ExosuitStates.STANDBY
            # Start main control loop
            self.thread.start()

        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")
            self._cleanup()

    def _cleanup(self) -> None:
        """Clean up the soft exoskeleton."""
        logger.info("Cleaning up exosuit.")
        self._status = ExosuitStates.STOPPED
        GPIO.cleanup()
        self.imu_left.stop()
        self.imu_right.stop()
        self.motor_left.close()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        logger.success("Exosuit shutdown.")

    def _loop(self) -> None:
        """Run main control loop."""
        while True:
            while self._status == ExosuitStates.RUNNING:
                try:
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

                    self.motor_left.set_velocity(
                        convert_rad_per_sec_to_rpm(command_left)
                    )

                    # place holder
                    self.motor_right.append(
                        f"Motor command at timestamp {timestamp_right} is {convert_rad_per_sec_to_rpm(command_right)} ERPM.)"
                    )

                    time.sleep(1 / self.config.frequency)
                except TypeError as err:
                    logger.error(f"Failed getting data from the IMU: '{err}'.")
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

            while self._status == ExosuitStates.PRETENSIONING:
                try:
                    # TODO right motor
                    left_motor_torque = 0.85  # place holder, get real torque here
                    if left_motor_torque >= TensionConfig.motor_torque_limit:
                        self.motor_left.set_velocity(0)
                        time.sleep(TensionConfig.tensioning_timeout)
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

            time.sleep(EXOSUIT_STANDBY_INTERVAL)

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
            if detected_imus != 2:
                raise ValueError(f"Wrong number of IMUs detected: {detected_imus} != 2")
            for idx in range(2):  # Match each leg with the IMU according to IMUConfig
                manager = sensor_managers[idx]
                if (
                    not left_init
                    and manager.i2c_id == IMUConfig.left_leg_bus
                    and manager.imu_descriptor == IMUConfig.left_leg_descr
                ):
                    self.imu_left = manager
                    left_init = True
                    continue
                if (
                    manager.i2c_id == IMUConfig.right_leg_bus
                    and manager.imu_descriptor == IMUConfig.right_leg_descr
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
        # TODO: right motor
        try:
            if not self.motor_left.check_communication():
                logger.error("Motor not responding. Check power and connections.")
                return False
            return True
        except Exception as err:
            logger.error(f"Exosuit exception: '{err}'. Check Motor connections.")
            return False
