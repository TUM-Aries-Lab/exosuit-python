"""Common definitions for this module."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum, StrEnum
from pathlib import Path

import numpy as np
from imu_python.definitions import I2CBusID, IMUDescriptor

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

DEFAULT_EXOSUIT_FREQUENCY_HZ = 100

THREAD_JOIN_TIMEOUT = 2.0

SWITCH_EVENT_HANDLER_INTERVAL = 0.5
EXOSUIT_STANDBY_INTERVAL = 0.1


@dataclass(frozen=True)
class TensionConfig:
    """Configurations for tensioning."""  # TODO: unit

    tensioning_velocity: int = 3
    motor_torque_limit: float = 0.85  # tensioned when motor_torque >= 0.85
    tensioning_timeout: float = 1.0  # in sec
    torque_check_interval: float = 0.1  # in sec


@dataclass(frozen=True)
class IMUConfig:
    """IMU configuration dataclass containing busID, IMU name and index for each leg."""

    left_leg_bus: int = I2CBusID.bus_7
    left_leg_descr: IMUDescriptor = field(
        default_factory=lambda: IMUDescriptor(name="MOCK", index=0)
    )
    right_leg_bus: int = I2CBusID.bus_7
    right_leg_descr: IMUDescriptor = field(
        default_factory=lambda: IMUDescriptor(name="MOCK", index=1)
    )  # TODO: set actual IMUs


# Switch pins
POWER_SWITCH = 29
TENSION_SWITCH = 31
MODE_SWITCH_1 = 32
MODE_SWITCH_2 = 33

GPIO_SWITCH_BOUNCETIME = 50  # TODO: test this threshold


# switch signals - used as 'getattr' keys for GPIO
class SwitchStates(StrEnum):
    """Enum to match switch states with electrical signals."""

    ON = "HIGH"
    OFF = "LOW"


BOTH = "BOTH"


class ExosuitStates(IntEnum):
    """Enum for exosuit states."""

    INITIALIZING = 0
    STANDBY = 1
    PRETENSIONING = 2
    RUNNING = 3
    STOPPED = 4


class InclinationModes(IntEnum):
    """Enum for operation modes for different inclinations."""

    LEVEL_GROUND = 0
    UPHILL = 1
    DOWNHILL = 2


@dataclass
class ModeSwitchStates:
    """Data class representing the states of the mode switch."""

    switch_1: SwitchStates
    switch_2: SwitchStates


# mode switch wiring:
MODE_SWITCH_LOGIC: dict[InclinationModes, ModeSwitchStates] = {
    InclinationModes.UPHILL: ModeSwitchStates(
        switch_1=SwitchStates.ON,
        switch_2=SwitchStates.OFF,
    ),
    InclinationModes.DOWNHILL: ModeSwitchStates(
        switch_1=SwitchStates.OFF,
        switch_2=SwitchStates.ON,
    ),
    InclinationModes.LEVEL_GROUND: ModeSwitchStates(
        switch_1=SwitchStates.OFF,
        switch_2=SwitchStates.OFF,
    ),
}
