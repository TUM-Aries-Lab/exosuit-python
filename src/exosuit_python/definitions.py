"""Common definitions for this module."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum, StrEnum
from pathlib import Path
from typing import Any

import numpy as np
from imu_python.definitions import MOCK_NAME, I2CBusID, IMUDescriptor

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

    # Both legs pull positive. The rig's motors are mounted mirrored, which
    # is why it needs TENSION_VEL_LEFT = +3.0 against TENSION_VEL_RIGHT =
    # -3.0; this suit's are not. Bench-checked on 2026-09-18: a positive
    # command shortened the cable on both motors, so a negative one here would
    # have paid the right tendon out, and its chart would have run to the STOP
    # hold without ever building tension.
    #
    # Nothing ties this to the assist path's own mirroring, which lives in
    # hip-controller's right_limb_reverse and is set independently -- see the
    # note there. The two are separate because tensioning drives both tendons
    # the same way while assist drives the hips anti-phase.
    tensioning_velocity_left_rad_per_sec: float = 3.0
    tensioning_velocity_right_rad_per_sec: float = 3.0

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


@dataclass(frozen=True)
class MotorSaturation:
    """Command limits applied before a frame is packed.

    Taken unchanged from ``motor_control.py``'s MOTOR_SAT_* values, which the
    rig applies inside pack_mit_command(). They are deliberately tighter than
    the protocol window: the rig's own MIT range is +/-45 rad/s while it
    saturates at 41.87, which is 399.8 output RPM -- a 400 RPM mechanical
    limit rather than a protocol one. On the AK80-6 that is half its rated
    800 RPM, so the same number stays conservative on this motor.

    This matters more than it used to. The assist command used to reach the
    motor divided by 126, so a controller excursion arrived harmless; now it
    arrives at full magnitude, and the only remaining limit would be the
    motor profile's own +/-76 rad/s.
    """

    position_rad: tuple[float, float] = (-12.5, 12.5)
    velocity_rad_per_sec: tuple[float, float] = (-41.87, 41.87)
    kp: tuple[float, float] = (0.0, 500.0)
    kd: tuple[float, float] = (0.0, 5.0)
    torque_nm: tuple[float, float] = (-9.0, 9.0)


class TensionState(IntEnum):
    """States of the pre-tensioning chart, ported from ``motor_control.py``."""

    RESET = 0
    TENSIONING = 1
    STOP = 2
    DISABLE = 3


@dataclass(frozen=True)
class IMUConfig:
    """IMU configuration dataclass containing busID, IMU name and index for each leg."""

    # The real sensors, as detect_and_create() reports them on this rig. A
    # leg matches only when bus, name and index all agree, and the placeholder
    # these replaced said name="MOCK", so nothing could ever match and both
    # legs failed initialisation with the IMUs sitting there detected.
    #
    # Note the two are on *different* buses. imu-python names a sensor
    # "{imu_name}_{imu_index}_{bus_id}", so the detected LSM6DSOX_LIS3MDL_0_1
    # and LSM6DSOX_LIS3MDL_1_7 are index 0 on bus 1 and index 1 on bus 7.
    # Pinning both to one bus was the second half of the mismatch.
    #
    # TO CONFIRM: which physical leg each sensor is on. Bus and index say
    # nothing about anatomy, so this assignment is a guess until someone
    # flexes one hip and checks which raw angle trace moves.
    left_leg_bus: int = I2CBusID.bus_1
    left_leg_descr: IMUDescriptor = field(
        default_factory=lambda: IMUDescriptor(name="LSM6DSOX_LIS3MDL", index=0)
    )
    right_leg_bus: int = I2CBusID.bus_7
    right_leg_descr: IMUDescriptor = field(
        default_factory=lambda: IMUDescriptor(name="LSM6DSOX_LIS3MDL", index=1)
    )

    @classmethod
    def for_mock_devices(cls) -> IMUConfig:
        """Return the config imu-python's mock sensors satisfy.

        The mocks are both called MOCK_NAME and share one bus, so the real
        hardware's names and split buses can never match them. Before the real
        values were filled in, this one config described the mocks -- which is
        why the mock tests passed while the sensors on the bench went
        unmatched. They describe different hardware and cannot be one value.

        :return: An IMU config matching the mock factory's sensors.
        :rtype: IMUConfig
        """
        return cls(
            left_leg_bus=I2CBusID.bus_7,
            left_leg_descr=IMUDescriptor(name=MOCK_NAME, index=0),
            right_leg_bus=I2CBusID.bus_7,
            right_leg_descr=IMUDescriptor(name=MOCK_NAME, index=1),
        )


# Switch pins, named in TEGRA_SOC mode with the BOARD number that was
# verified against the wiring kept alongside.
#
# The mode is not a free choice. Jetson.GPIO allows one mode per process and
# raises "A different mode has already been set!" on a second, different one.
# Importing any Adafruit register-based IMU driver pulls in Blinka, whose
# tegra/t234/pin.py calls setmode(TEGRA_SOC) at import time -- by way of
# circuitpython_typing.device_drivers, which imports SPIDevice at module level
# purely for type annotations, with no TYPE_CHECKING guard. So BOARD here and
# an IMU driver anywhere cannot coexist, whichever loads first.
#
# Matching Blinka resolves it: setmode() only raises when the modes differ, so
# with TEGRA_SOC set here Blinka's call becomes a no-op. The names come from
# Jetson.GPIO's own pin table for JETSON_ORIN_NANO, read off the same
# definition row as the BOARD number, so they are a translation and not a
# second source of truth.
# Typed Any deliberately. Jetson.GPIO annotates a channel as int, which
# describes BOARD and BCM; in TEGRA_SOC mode the channel is the pin's name and
# the library accepts it, because its channel table is keyed by whatever the
# selected mode uses. Any keeps that honest rather than claiming an int these
# are not, and avoids scattering ignores over every call site.
SwitchChannel = Any

OPERATION_SWITCH: SwitchChannel = "GP88_PWM1"  # BOARD 15
TENSION_SWITCH: SwitchChannel = "GP167"  # BOARD 7
MODE_SWITCH_1: SwitchChannel = "GP65"  # BOARD 29
MODE_SWITCH_2: SwitchChannel = "GP113_PWM7"  # BOARD 32

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
