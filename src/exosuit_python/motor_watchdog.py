"""Detect a motor that has dropped out of MIT mode, and put it back.

A CubeMars motor that trips its own protection stops acting on commands but
keeps answering them. Telemetry arrives at the usual rate and the fault code
stays zero, so nothing upstream notices: on 2026-09-21 the right motor stopped
at t=8.84 s and the control loop went on commanding it up to 44 rad/s for the
next 16 seconds, against a shaft that never moved again.

What gives it away is the *torque*. A motor held by a load reports high torque.
This one reported 0.19 N*m on average while being asked for 15 rad/s, which no
loaded motor does -- it is the signature of a motor that is powered, listening,
and not in motor mode.

The rig's equivalent, ``motor_control.py``'s auto-recovery, re-sends
ENTER_MOTOR_MODE when feedback goes *stale*. That would not have caught this:
feedback never stopped.
"""

from enum import Enum, auto

from exosuit_python.definitions import MotorWatchdogConfig


class WatchdogVerdict(Enum):
    """What the caller should do about this leg on this tick."""

    #: Nothing is wrong.
    HEALTHY = auto()
    #: The motor looks dropped out. Re-enable it and command zero this tick.
    RECOVER = auto()
    #: Re-enabling has been tried enough times. Stop commanding this leg.
    GIVE_UP = auto()


class MotorDropoutWatchdog:
    """Watch one motor for the commanded-but-inert signature.

    The test is deliberately three-sided, because each part rules out a
    different innocent explanation:

    * a command large enough that the motor should visibly move -- otherwise a
      motor correctly holding still at zero command looks identical;
    * a speed near zero -- the motor is not doing it;
    * a *mean* torque near zero over the whole stretch -- it is not being
      prevented from doing it. This is what separates a dropout from a stall
      against a jammed tendon, which is a mechanical fault needing a human, not
      a re-enable.

    The measured margin is wide. While driving healthily the longest run of
    "commanded above 5 rad/s and moving under 1 rad/s" was 6 ticks; the dead
    motor held it for 16 seconds.
    """

    def __init__(self, config: MotorWatchdogConfig | None = None) -> None:
        """Build a watchdog for one motor.

        :param config: Detection thresholds. Defaults to
            ``MotorWatchdogConfig()``.
        :return: None
        """
        self._config = config if config is not None else MotorWatchdogConfig()
        self._suspect_ticks = 0
        self._torque_sum = 0.0
        self._attempts = 0
        self._given_up = False

    @property
    def recovery_attempts(self) -> int:
        """How many times this motor has been re-enabled during the session."""
        return self._attempts

    def reset(self) -> None:
        """Forget everything, including the give-up latch.

        Called at the start of a session, so a leg written off in one run is
        given a fresh chance in the next rather than staying dead until the
        process restarts.

        :return: None
        """
        self._suspect_ticks = 0
        self._torque_sum = 0.0
        self._attempts = 0
        self._given_up = False

    def step(
        self,
        command_rad_per_sec: float,
        speed_rad_per_sec: float,
        torque_nm: float,
    ) -> WatchdogVerdict:
        """Judge one tick of this motor's behaviour.

        :param command_rad_per_sec: What the loop asked for, in rad/s.
        :param speed_rad_per_sec: What the motor reports it is doing, in rad/s.
        :param torque_nm: What the motor reports it is producing, in N*m.
        :return: What the caller should do about this leg.
        :rtype: WatchdogVerdict
        """
        if not self._config.enabled:
            # Fully inert, including the give-up latch: switching it off must
            # hand the leg straight back to the loop, not leave it written off
            # by a verdict reached before the switch was thrown.
            return WatchdogVerdict.HEALTHY

        if self._given_up:
            return WatchdogVerdict.GIVE_UP

        commanded = abs(command_rad_per_sec) > self._config.min_command_rad_per_sec
        inert = abs(speed_rad_per_sec) < self._config.max_speed_rad_per_sec
        if not (commanded and inert):
            self._suspect_ticks = 0
            self._torque_sum = 0.0
            return WatchdogVerdict.HEALTHY

        self._suspect_ticks += 1
        self._torque_sum += abs(torque_nm)
        if self._suspect_ticks < self._config.ticks_before_fault:
            return WatchdogVerdict.HEALTHY
        return self._verdict_on_a_full_stretch()

    def _verdict_on_a_full_stretch(self) -> WatchdogVerdict:
        """Decide what a completed stretch of suspect ticks means.

        Reached only once the motor has looked commanded-but-inert for the
        whole window, so the counters are cleared here whatever the answer.

        :return: What the caller should do about this leg.
        :rtype: WatchdogVerdict
        """
        mean_torque = self._torque_sum / self._suspect_ticks
        self._suspect_ticks = 0
        self._torque_sum = 0.0

        if mean_torque >= self._config.max_mean_torque_nm:
            # Being held, not dropped out. A re-enable would not help, and the
            # loop is already commanding into it; leave it to the operator.
            return WatchdogVerdict.HEALTHY

        self._attempts += 1
        if self._attempts > self._config.max_recovery_attempts:
            self._given_up = True
            return WatchdogVerdict.GIVE_UP
        return WatchdogVerdict.RECOVER
