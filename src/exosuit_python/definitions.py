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

# Fallback wake-up for the switch handler. Operation and tension switches now
# wake it on their GPIO edge, so this only bounds how long a MODE_SWITCH_1/2
# change waits -- those are polled, not edge-detected.
SWITCH_EVENT_HANDLER_INTERVAL = 0.5
EXOSUIT_STANDBY_INTERVAL = 0.1


@dataclass(frozen=True)
class TensionConfig:
    """Pre-tensioning parameters, ported from ``motor_control.py``.

    The reference implementation is the Simulink port in the LocomotionMode
    repository (``motor_control.py``, Subsystem3): the torque feedback is
    low-pass filtered, rectified, and handed to a four-state chart that pulls
    the tendon until the torque crosses a fixed threshold.

    This resolves the old "TODO: unit". The units are the MIT force-control
    protocol's own -- velocity in rad/s at the output shaft, torque in N*m.
    ``CubeMarsAK806v2CAN.get_current()`` returns the MIT feedback field, which
    motor-python decodes against the motor profile's ``t_min``/``t_max``, so
    the value is a torque in N*m in spite of the name. No torque constant is
    involved, which is why none appears here.
    """

    # Velocity command per leg. The legs are mirrored, so the signs differ --
    # TENSION_VEL_LEFT = +3.0 and TENSION_VEL_RIGHT = -3.0 upstream.
    tensioning_velocity_left_rad_per_sec: float = 3.0
    tensioning_velocity_right_rad_per_sec: float = -3.0

    # |LPF(torque)| at which tensioning stops -- TORQUE_THRESHOLD.
    torque_threshold_nm: float = 0.85

    # Seconds spent in STOP before DISABLE -- STOP_HOLD_TIME / after(1,sec).
    stop_hold_time: float = 1.0

    # MIT damping gain accompanying the velocity command -- MOTOR_KD_CMD.
    # AK80_6_MOTOR_SPEC.mit_velocity_kd carries the same 1.0 for the same
    # reason, but it is marked UNVERIFIED UNDER LOAD there: the bench run
    # behind it was free-shaft with the tendon disconnected. Tensioning is a
    # loaded condition, so confirm this on the assembled suit.
    mit_velocity_kd: float = 1.0

    # Torque-feedback low-pass filter -- TENSION_LPF_WN / TENSION_LPF_ZT.
    torque_lpf_cutoff_rad_per_sec: float = 25.0
    torque_lpf_damping_ratio: float = 1.0


@dataclass(frozen=True)
class MotorCommandConfig:
    """MIT command gains for the running (assist) path.

    These mirror ``motor_control.py``'s MOTOR_KP_CMD / MOTOR_KD_CMD /
    MOTOR_TORQUE_CMD: the assist command is velocity-only, so the position
    gain and the feed-forward torque are both zero and Kd alone closes the
    loop on speed.

    ``velocity_kd`` is also handed to the motor at construction, so the
    package's own ``set_velocity`` path and the direct ``set_mit_mode`` calls
    here cannot drift apart. It starts equal to ``AK80_6_MOTOR_SPEC``'s value,
    which was chosen for the same reason; the copy lives here so both of this
    repo's Kd knobs -- this one and TensionConfig.mit_velocity_kd -- are
    visible in one place rather than one of them hiding in the package.
    """

    kp: float = 0.0
    velocity_kd: float = 1.0
    torque_ff_nm: float = 0.0


class TensionState(IntEnum):
    """States of the pre-tensioning chart, ported from ``motor_control.py``."""

    RESET = 0
    TENSIONING = 1
    STOP = 2
    DISABLE = 3


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
OPERATION_SWITCH = 15
TENSION_SWITCH = 7
MODE_SWITCH_1 = 29
MODE_SWITCH_2 = 32

# Debounce window for the two edge-detected switches, in milliseconds.
#
# In Jetson.GPIO this is a rejection window, not a delay: an accepted edge
# fires its callback immediately, and every further edge on that pin is
# dropped for this long (Jetson/GPIO/gpio_event.py, the `lastcall` check). It
# therefore contributes nothing to switch latency. What it costs is that a
# genuine transition arriving inside the window is lost outright rather than
# deferred.
#
# That used to be able to latch a stale state, because the callbacks sampled
# the pin themselves: an edge caught mid-bounce could read the pre-settled
# level and suppress the settling edge that followed. The callbacks no longer
# sample -- _switch_event_handler reads the settled level and re-reads it
# every SWITCH_EVENT_HANDLER_INTERVAL regardless -- so a dropped edge now
# self-corrects within one pass.
#
# The value only has to outlast contact bounce (a few ms) while staying well
# below the shortest deliberate actuation, so it is no longer critical.
GPIO_SWITCH_BOUNCETIME = 50


# CAN node IDs. Same values as motor_control.py's CAN_ID_LEFT / CAN_ID_RIGHT,
# and confirmed against this suit's own wiring on 2026-09-18 -- the AK60-6 rig
# and the AK80-6 suit use the same node IDs, so these are checked rather than
# merely inherited.
MOTOR_CAN_ID_LEFT = 4
MOTOR_CAN_ID_RIGHT = 3


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


# Inclination mode -> hip-controller locomotion class_id.
#
# `WalkOnController.set_locomotion_mode(class_id)` fans one integer out across
# the whole controller: amplitude parameters, the per-mode SOGI-FLL tuning, and
# the motion-mapping table. It replaces reaching into
# `controller.amplitude_modulation.set_mode(...)`, which only retuned the first
# of those three.
#
# The mode switches select the locomotion mode by hand -- the same thing the
# TCN classifier produces automatically in the LocomotionMode project -- so
# UPHILL means stair ascent here, not a ramp. The `InclinationModes` names
# predate that usage.
#
# The values agree numerically with hip-controller's class_id today, but the
# two enumerations live in different packages and are mapped explicitly so
# either can be renumbered without silently changing which tuning is selected.
LOCOMOTION_CLASS_IDS: dict[InclinationModes, int] = {
    InclinationModes.LEVEL_GROUND: 0,
    InclinationModes.UPHILL: 1,
    InclinationModes.DOWNHILL: 2,
}

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
