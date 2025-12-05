"""Sample doc string."""

from dataclasses import dataclass

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

        self.imu_left = "imu"  # place holder
        self.imu_right = "imu"  # place holder

    def run(self):
        """Start the soft exoskeleton."""
        self.start()

    def start(self):
        """Start the IMUs and Motors."""
        logger.info(f"Starting Exosuit at '{self.config.frequency}' Hz.")
        try:
            logger.debug("Starting IMUs")
            logger.debug("Starting Motors")
            logger.debug("Starting Controller")
            self._is_running = True
        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")

    def cleanup(self):
        """Clean up the soft exoskeleton."""
        logger.info("Cleaning up exosuit.")
        self._is_running = False
        logger.success("Exosuit shutdown.")
