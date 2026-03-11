"""Exosuit configuration."""

import threading
import time
from dataclasses import dataclass

from hip_controller.control.app import WalkOnController
from hip_controller.definitions import SensorSignal
from imu_python.definitions import I2CBusID
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from loguru import logger
from motor_python.cube_mars_motor import CubeMarsAK606v3

from exosuit_python.definitions import THREAD_JOIN_TIMEOUT
from exosuit_python.utils import convert_rad_per_sec_to_rpm


@dataclass
class ExosuitConfig:
    """Exosuit configuration."""

    frequency: float


class Exosuit:
    """Tendon-based soft exoskeleton."""

    def __init__(self, config: ExosuitConfig) -> None:
        """Initialize the exosuit.

        :param config: Exosuit configuration
        """
        self.config = config

        self._is_running: bool = False
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        self.imu_hip: IMUManager
        self.imu_left: IMUManager
        self.imu_right: IMUManager
        self.controller_left = WalkOnController(reverse=False)
        self.controller_right = WalkOnController(reverse=True)
        self.motor_left: CubeMarsAK606v3 = CubeMarsAK606v3()
        self.motor_left_position_degrees: float = 0.0  # place holder
        self.motor_right = ["Motor right."]  # place holder

        self.imu_initialized: bool = self._initialize_imus()
        self.motors_initialized: bool = self._initialize_motors()

    def run(self):
        """Start the soft exoskeleton."""
        if not self.imu_initialized:
            logger.error("IMU initialization failed. Exosuit not started.")
            return
        if not self.motors_initialized:
            logger.error("Motor initialization failed. Exosuit not started.")
            return

        self.start()

    def start(self):
        """Start the IMUs and Motors."""
        logger.info(f"Starting Exosuit at '{self.config.frequency}' Hz.")
        try:
            logger.debug("Starting IMUs")
            self.imu_hip.start()
            self.imu_left.start()
            self.imu_right.start()
            logger.debug("Starting Motors")
            logger.debug("Starting Controller")
            self._is_running = True

            # Start main control loop
            self.thread.start()

        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")
            self.cleanup()

    def cleanup(self):
        """Clean up the soft exoskeleton."""
        logger.info("Cleaning up exosuit.")
        self.imu_hip.stop()
        self.imu_left.stop()
        self.imu_right.stop()
        self.motor_left.close()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        self._is_running = False
        logger.success("Exosuit shutdown.")

    def _loop(self) -> None:
        """Run main control loop."""
        while self._is_running:
            try:
                timestamp_right = self.imu_right.get_data().timestamp
                signal_right = SensorSignal(
                    angle_rad=self.imu_right.get_data().quat.to_euler(seq="xyz").z,
                    velocity_rad_per_sec=self.imu_right.get_data().raw_data.gyro.z,
                )
                command_right = self.controller_right.step(
                    timestamp=timestamp_right, curr_signal=signal_right
                )

                timestamp_left = self.imu_left.get_data().timestamp
                signal_left = SensorSignal(
                    angle_rad=self.imu_left.get_data().quat.to_euler(seq="xyz").z,
                    velocity_rad_per_sec=self.imu_right.get_data().raw_data.gyro.z,
                )
                command_left = self.controller_left.step(
                    timestamp=timestamp_left, curr_signal=signal_left
                )

                self.motor_left.set_velocity(convert_rad_per_sec_to_rpm(command_left))

                # place holder
                self.motor_right.append(
                    f"Motor command at timestamp {timestamp_right} is {convert_rad_per_sec_to_rpm(command_right)} ERPM.)"
                )

                time.sleep(1 / self.config.frequency)
            except Exception as err:
                logger.error(f"Exosuit control loop exception: '{err}'.")

    def _initialize_imus(self) -> bool:
        """Initialize IMUs.

        :return: True if successful, False otherwise
        """
        try:
            sensor_managers_hip = IMUFactory.detect_and_create(
                i2c_id=I2CBusID.bus_1,
                log_data=False,
            )
            sensor_managers_legs = IMUFactory.detect_and_create(
                i2c_id=I2CBusID.bus_7,
                log_data=False,
            )
            self.imu_hip = sensor_managers_hip[0]
            self.imu_left = sensor_managers_legs[0]
            self.imu_right = sensor_managers_legs[1]
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
