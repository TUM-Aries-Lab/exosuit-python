"""Sample doc string."""

from dataclasses import dataclass

from imu_python.definitions import I2CBusID
from imu_python.factory import IMUFactory
from loguru import logger


@dataclass
class ExosuitConfig:
    """Exosuit configuration."""

    frequency: float


class Exosuit:
    """Tendon-based soft exoskeleton."""

    def __init__(self, config: ExosuitConfig):
        self.config = config

        self._is_running: bool = False

        self.motor_left = "motor"  # place holder
        self.motor_right = "motor"  # place holder

        try:
            logger.debug("Starting IMUs")
            sensor_managers_l = IMUFactory.detect_and_create(
                i2c_id=I2CBusID.left,
                log_data=False,
            )
            sensor_managers_r = IMUFactory.detect_and_create(
                i2c_id=I2CBusID.right,
                log_data=False,
            )
            self.imu_center = sensor_managers_l[0]
            self.imu_left = sensor_managers_r[0]
            self.imu_right = sensor_managers_r[1]
        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")

    def run(self):
        """Start the soft exoskeleton."""
        self.start()

    def start(self):
        """Start the IMUs and Motors."""
        logger.info(f"Starting Exosuit at '{self.config.frequency}' Hz.")
        try:
            logger.debug("Starting IMUs")
            sensor_managers_l = IMUFactory.detect_and_create(
                i2c_id=I2CBusID.left,
                log_data=False,
            )
            sensor_managers_r = IMUFactory.detect_and_create(
                i2c_id=I2CBusID.right,
                log_data=False,
            )
            self.imu_center = sensor_managers_l[0]
            self.imu_left = sensor_managers_r[0]
            self.imu_right = sensor_managers_r[1]
            self.imu_center.start()
            self.imu_left.start()
            self.imu_right.start()
            logger.debug("Starting Motors")
            logger.debug("Starting Controller")
            self._is_running = True
        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")

    def cleanup(self):
        """Clean up the soft exoskeleton."""
        logger.info("Cleaning up exosuit.")
        self.imu_center.stop()
        self.imu_left.stop()
        self.imu_right.stop()
        self._is_running = False
        logger.success("Exosuit shutdown.")
