"""Common definitions for this module."""

from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from imu_python.definitions import IMUDescriptor

np.set_printoptions(precision=3, floatmode="fixed", suppress=True)


# --- Directories ---
ROOT_DIR: Path = Path("src").parent
DATA_DIR: Path = ROOT_DIR / "data"
RECORDINGS_DIR: Path = DATA_DIR / "recordings"
LOG_DIR: Path = DATA_DIR / "logs"

# Default encoding
ENCODING: str = "utf-8"

DATE_FORMAT = "%Y-%m-%d_%H-%M-%S"

DUMMY_VARIABLE = "dummy_variable"


@dataclass
class LogLevel:
    """Log level."""

    trace: str = "TRACE"
    debug: str = "DEBUG"
    info: str = "INFO"
    success: str = "SUCCESS"
    warning: str = "WARNING"
    error: str = "ERROR"
    critical: str = "CRITICAL"

    def __iter__(self):
        """Iterate over log levels."""
        return iter(asdict(self).values())


DEFAULT_LOG_LEVEL = LogLevel.info
DEFAULT_LOG_FILENAME = "log_file"

THREAD_JOIN_TIMEOUT = 2.0

SWITCH_EVENT_HANDLER_INTERVAL = 0.5
EXOSUIT_STANDBY_INTERVAL = 0.1


@dataclass(frozen=True)
class TensionConfig:
    """Configurations for tensioning."""

    tensioning_velocity: int = 3
    motor_torque_limit: float = 0.85  # tensioned when motor_torque >= 0.85

    tensioning_timeout: float = 1.0  # in sec


@dataclass(frozen=True)
class IMUConfig:
    """IMU configuration dataclass containing busID, IMU name and index for each leg."""

    left_leg_bus: int = 7
    left_leg_descr: IMUDescriptor = field(
        default_factory=lambda: IMUDescriptor(name="BNO055", index=0)
    )
    right_leg_bus: int = 7
    right_leg_descr: IMUDescriptor = field(
        default_factory=lambda: IMUDescriptor(name="BNO055", index=1)
    )


# Switch pins
POWER_SWITCH = 7  # TODO: placeholder
TENSION_SWITCH = 15  # TODO: placeholder

GPIO_SWITCH_BOUNCETIME = 50  # TODO: test this threshold
