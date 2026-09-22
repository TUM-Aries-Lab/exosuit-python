"""Tests for the dropped-out-motor watchdog.

The numbers here are the ones measured on 2026-09-21, when the right motor
stopped at t=8.84 s and was commanded for another 16 seconds without anyone
noticing.
"""

from dataclasses import replace

from exosuit_python.definitions import MotorWatchdogConfig
from exosuit_python.motor_watchdog import MotorDropoutWatchdog, WatchdogVerdict

CONFIG = MotorWatchdogConfig()

# Measured while the right motor was dead: commanded hard, not moving, and
# producing essentially nothing.
DEAD = dict(command_rad_per_sec=14.9, speed_rad_per_sec=0.067, torque_nm=0.186)
# Measured while both motors were driving properly.
DRIVING = dict(command_rad_per_sec=20.0, speed_rad_per_sec=13.8, torque_nm=1.97)


def _feed(watchdog, ticks, **sample):
    """Step the watchdog and return the verdicts it gave."""
    return [watchdog.step(**sample) for _ in range(ticks)]


def test_a_driving_motor_is_left_alone():
    """Normal operation must never be interrupted."""
    watchdog = MotorDropoutWatchdog()

    verdicts = _feed(watchdog, 500, **DRIVING)

    assert set(verdicts) == {WatchdogVerdict.HEALTHY}


def test_a_still_motor_at_rest_is_left_alone():
    """No command means no expectation of movement, so nothing is wrong."""
    watchdog = MotorDropoutWatchdog()

    verdicts = _feed(
        watchdog, 500, command_rad_per_sec=0.0, speed_rad_per_sec=0.0, torque_nm=0.0
    )

    assert set(verdicts) == {WatchdogVerdict.HEALTHY}


def test_the_dropout_is_caught():
    """The 2026-09-21 failure, which ran for 16 s unnoticed."""
    watchdog = MotorDropoutWatchdog()

    verdicts = _feed(watchdog, CONFIG.ticks_before_fault, **DEAD)

    assert verdicts[-1] is WatchdogVerdict.RECOVER
    assert set(verdicts[:-1]) == {WatchdogVerdict.HEALTHY}


def test_a_stalled_motor_is_not_re_enabled():
    """High torque means it is being held, which a re-enable cannot fix.

    A jammed tendon is a mechanical fault wanting a human. Re-enabling into it
    would drive a motor at full command against the jam.
    """
    watchdog = MotorDropoutWatchdog()

    verdicts = _feed(
        watchdog,
        CONFIG.ticks_before_fault * 3,
        command_rad_per_sec=14.9,
        speed_rad_per_sec=0.0,
        torque_nm=8.0,
    )

    assert set(verdicts) == {WatchdogVerdict.HEALTHY}


def test_a_brief_stall_does_not_trip_it():
    """The longest such stretch while driving healthily was 6 ticks."""
    watchdog = MotorDropoutWatchdog()

    for _ in range(50):
        stalled = _feed(watchdog, 6, **DEAD)
        assert set(stalled) == {WatchdogVerdict.HEALTHY}
        # One good tick clears it, as a real stride would.
        assert watchdog.step(**DRIVING) is WatchdogVerdict.HEALTHY


def test_it_stops_trying_after_enough_attempts():
    """A motor that keeps dropping out is failing, not glitching."""
    watchdog = MotorDropoutWatchdog()

    recoveries = 0
    for _ in range(CONFIG.max_recovery_attempts + 5):
        verdicts = _feed(watchdog, CONFIG.ticks_before_fault, **DEAD)
        if verdicts[-1] is WatchdogVerdict.RECOVER:
            recoveries += 1

    assert recoveries == CONFIG.max_recovery_attempts
    assert watchdog.step(**DEAD) is WatchdogVerdict.GIVE_UP


def test_giving_up_latches():
    """Once written off, the leg stays off for the session, even if it twitches."""
    watchdog = MotorDropoutWatchdog()
    for _ in range(CONFIG.max_recovery_attempts + 1):
        _feed(watchdog, CONFIG.ticks_before_fault, **DEAD)

    assert watchdog.step(**DEAD) is WatchdogVerdict.GIVE_UP
    assert watchdog.step(**DRIVING) is WatchdogVerdict.GIVE_UP


def test_reset_gives_the_leg_another_session():
    """A new session starts clean rather than inheriting the last one's verdict."""
    watchdog = MotorDropoutWatchdog()
    for _ in range(CONFIG.max_recovery_attempts + 1):
        _feed(watchdog, CONFIG.ticks_before_fault, **DEAD)
    assert watchdog.step(**DEAD) is WatchdogVerdict.GIVE_UP

    watchdog.reset()

    assert watchdog.recovery_attempts == 0
    assert watchdog.step(**DRIVING) is WatchdogVerdict.HEALTHY


def test_it_is_on_by_default():
    """The flag ships enabled, so a run gets the protection without asking."""
    assert CONFIG.enabled is True


def test_switching_it_off_makes_it_inert():
    """Disabled, it must never intervene however dead the motor looks."""
    watchdog = MotorDropoutWatchdog(replace(CONFIG, enabled=False))

    verdicts = _feed(watchdog, CONFIG.ticks_before_fault * 10, **DEAD)

    assert set(verdicts) == {WatchdogVerdict.HEALTHY}
    assert watchdog.recovery_attempts == 0


def test_switching_it_off_releases_a_leg_it_had_written_off():
    """The give-up latch must not outlive the switch.

    Otherwise turning the watchdog off to rule it out would leave the leg it
    had already condemned still uncommanded, and look like the watchdog was
    not the problem.
    """
    watchdog = MotorDropoutWatchdog()
    for _ in range(CONFIG.max_recovery_attempts + 1):
        _feed(watchdog, CONFIG.ticks_before_fault, **DEAD)
    assert watchdog.step(**DEAD) is WatchdogVerdict.GIVE_UP

    watchdog._config = replace(CONFIG, enabled=False)

    assert watchdog.step(**DEAD) is WatchdogVerdict.HEALTHY
