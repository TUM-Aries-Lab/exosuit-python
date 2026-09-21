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

    # How old the cached CAN feedback may be before the torque is fetched with
    # a blocking request instead. The keep-alive thread refreshes it at the
    # motor control rate, so it is normally a few milliseconds old; this only
    # has to be loose enough not to trip on ordinary jitter.
    torque_staleness_s: float = 0.05

    # Ceiling on the measured time step handed to the filter and the STOP
    # timer, as a multiple of the nominal period. Using real elapsed time is
    # what keeps them honest when the loop runs late, but a long stall would
    # otherwise push a step of hundreds of milliseconds into a 25 rad/s filter
    # and rail it -- the same failure the SOGI dt clamp exists for upstream.
    max_time_step_periods: float = 3.0


@dataclass(frozen=True)
class PositionLoopConfig:
    """The outer position loop -- gains for hip-controller's ``PIDController``.

    Read off ``matlab/Control_ML_Stairs_IMUbased_developer.slx`` in the
    LocomotionMode repository, blocks PID1 (SID 7524) and PID2 (SID 7551),
    which are identical. The block feeding their ``Ref`` inport is named
    **MOTOR POSITION REFERENCE GENERATOR** and its outport ``Ref motion``: what
    ``WalkOnController.step`` returns is a motor *position* reference in
    radians, not a velocity. Their ``Actual Motion`` inport is the motor's own
    measured position, by way of the model's unwrapping block.

        u = kp * (reference - measured) + ki * integral - damping_gain * LPF(u_previous)'

    The derivative term is not the derivative of the error: ``Gain7`` is fed
    from the second outport of a second-order low-pass filter whose input is
    the PID's previous output, and is subtracted. That is velocity-feedback
    damping, and it is the reason a plain P controller in its place would
    behave differently.

    hip-controller ships this block as ``PIDController`` and, until it was
    wired up, nothing imported it. The reference went straight to the motor as
    a velocity, which integrates it. Because the reference spends most of a
    stride positive -- the tendon is pulled in during flexion and returns
    towards zero, so it never needs to go far negative -- that became a
    standing order to keep winding: a 2026-09-21 bench run wound 6.94 rad into
    the left spool and 5.39 into the right over 15.5 s and gave none of it
    back, which the wearer felt as a slow creep that held tension hard and
    answered the leg barely at all. Closed as a position loop on the rig, the
    same signal tracks: measured against reference, correlation 0.87 with 0.004
    rad of net drift across 23k samples.
    """

    # Gain4 -- KP_MAIN. The rig runs 16.0 and that is what this started at,
    # but the rig is an AK60-6: motor_control.py decodes P +/-12.5, V +/-45,
    # T +/-15, an exact match for AK60_6_V1_1_MIT_LIMITS, and its header cites
    # the AK60-6 manual. This suit is an AK80-6 -- 6.0 Nm rated against 3.0,
    # 12.0 peak against 9.0, 21 pole pairs against 14, and a much heavier
    # rotor.
    #
    # The gain does not survive that swap. It sets the velocity trajectory the
    # motor is asked to follow, and the torque to follow it is J * alpha, so a
    # bigger rotor draws proportionally more for the same command. Measured on
    # 2026-09-21 against the rig's no-classif ID_07 run, torque while the motor
    # was moving: 3.48 / 3.78 N*m here against 1.78 / 1.38 on the rig -- a
    # factor of 2 to 2.7 -- while torque at rest matched (0.58 / 0.86 against
    # 0.50 / 0.41), which is what isolates it to acceleration rather than
    # tension. The motor also reversed direction 33.9 / 20.7 times a second
    # against the rig's 15.4 / 14.3: it lags, overshoots, and comes back.
    #
    # 16 * (1.6 / 3.6) is about 7, so matching the rig's moving torque lands on
    # the Simulink model's original 8.0 -- which is also hip-controller's
    # PIDConfig default. Raise it back towards 16 if the assist feels weak or
    # tracking is loose; the two numbers bracket the useful range.
    proportional_gain: float = 8.0

    # Gain8 -- KI_MAIN. Zero, which disables the integral term outright. It is
    # carried rather than dropped because the state it integrates is what a
    # future tuning session would switch on.
    integral_gain: float = 0.0

    # Gain7 -- KD_INNER_MAIN, written in the model as the expression
    # 0.06-0.04, acting on the filtered derivative of the loop's own previous
    # output rather than on the derivative of the error.
    damping_gain: float = 0.02

    # PID_LPF_WN / PID_LPF_ZT, and the mask on the model's own filter block:
    # wn = 20 rad/s, zt = 1, x0 = 0.
    output_lpf_cutoff_rad_per_sec: float = 20.0
    output_lpf_damping_ratio: float = 1.0

    # Saturation is deliberately NOT applied inside the loop. In the model,
    # Sum5's output goes to the outport *and* to the damping filter, and
    # Saturation1 sits downstream in the motor block; motor_control.py does the
    # same, storing the unclipped u in _last_u and saturating in
    # pack_mit_command. hip-controller's PIDController clips before it stores,
    # so handing it output_limits would feed the damping term a different
    # signal from the rig's on exactly the ticks that saturate.
    # ``_command_velocity`` applies MotorSaturation.velocity_rad_per_sec --
    # the same +/-41.87 -- before the frame is packed, which is where the rig
    # applies it too.

    # Ceiling on the time step handed to the loop, in seconds. The model runs
    # fixed-step and has no equivalent; this loop does not, and a stall would
    # push a step of hundreds of milliseconds into a 20 rad/s filter and rail
    # it -- the same failure the SOGI dt clamp exists for upstream. Three
    # nominal periods at 100 Hz, matching TensionConfig's own ceiling.
    max_time_step: float = 0.03

    # The model's CHINESE CORRECTION block, which unwraps the motor position
    # between the CAN unpack and the PID's feedback inport: a step of more than
    # this between ticks is a rollover of the +/-12.5 rad position field, not
    # motion, and is accumulated out. The threshold is the model's Switch1.
    #
    # The rig gets away with a coarse threshold because it re-zeroes the
    # encoder when pre-tensioning ends and its references stay inside +/-10.47
    # rad, the reference generator's own saturation. This suit pulls further --
    # the 2026-09-21 run wound 13.47 rad into the right spool during
    # pre-tensioning alone, past the field's range before the assist even
    # began -- which is why the zeroing matters as much as the unwrapping.
    wrap_detection_threshold_rad: float = 20.0


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


@dataclass(frozen=True)
class IMUMounting:
    """How this suit's IMUs sit, where that changes the signal's sign.

    Bench-measured on 2026-09-18 by flexing each hip in turn and comparing the
    two channels against each other.

    ``gyro.z`` is read in the sensor's own frame, so a board mounted the other
    way round reports the opposite sign; the reported angle is not affected,
    because the fusion resolves orientation against gravity and a thigh at a
    given angle tilts the same way whichever way its board faces. On this suit
    that makes the two channels disagree on the right leg -- measured
    ``d(euler_y)/dt = +0.885 * gyro.z`` on the left against ``-0.898`` on the
    right -- and the controller reads angle against velocity as a phase
    portrait, so a disagreement reflects that leg into the wrong quadrant
    rather than failing outright.

    These signs bring the velocity back into agreement with the angle. They
    say nothing about the motors, which were measured separately and are not
    mirrored, nor about hip-controller's ``right_limb_reverse``.
    """

    gyro_sign_left: float = 1.0
    gyro_sign_right: float = -1.0


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
