"""Mock motor for CI testing."""

from loguru import logger


class MockMotor:
    """Mock motor class."""

    def __init__(self) -> None:
        pass

    def set_velocity(self, velocity_erpm: int) -> None:
        """Set mock motor velocity."""
        logger.debug(f"Velocity set: {velocity_erpm}")
        pass

    def close(self) -> None:
        """Close mock motor connection."""
        pass

    def check_communication(self) -> bool:
        """Check if motor can communicate."""
        return True

    def get_torque(self) -> int:
        # TODO: make this function name/param identical to the motor implementation
        """Get the torque of the motor."""
        return 0
