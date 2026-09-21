"""Tests for the outer position loop.

The property that matters is the one the bench run failed: a bounded reference
must produce bounded travel. Commanding the controller's reference straight to
the motor as a velocity integrates it, so a reference that is mostly positive
winds the spool in without ever giving it back.
"""

import math

from exosuit_python.definitions import PositionLoopConfig
from exosuit_python.position_loop import MotorPositionLoop

DT = 0.01
CONFIG = PositionLoopConfig()

# The MIT position field's range. The loop never needs the number -- it detects
# a rollover from the size of the step -- so it lives here, with the fake motor
# that produces one.
FIELD_LIMIT_RAD = 12.5


class _Spool:
    """A first-order shaft that follows a velocity command.

    Standing in for a motor that tracks its command well enough to close the
    loop -- enough to tell a loop that settles from one that runs away.
    """

    def __init__(self, wrap_rad: float = FIELD_LIMIT_RAD) -> None:
        self.position = 0.0
        self._wrap = wrap_rad

    def step(self, velocity_rad_per_sec: float) -> float:
        """Advance by one tick and report the position as the MIT field would."""
        self.position += velocity_rad_per_sec * DT
        span = 2.0 * self._wrap
        return (self.position + self._wrap) % span - self._wrap


def _follow(loop: MotorPositionLoop, spool: _Spool, reference_of, seconds: float):
    """Run the loop against the spool and return the commands it issued."""
    commands = []
    for tick in range(int(seconds / DT)):
        measured = spool.step(commands[-1] if commands else 0.0)
        commands.append(
            loop.step(
                reference_rad=reference_of(tick * DT),
                measured_rad=measured,
                enabled=True,
                time_difference=DT,
            )
        )
    return commands


def test_a_held_reference_is_reached_and_held():
    """A constant reference must settle the shaft on it, not drive past it."""
    loop, spool = MotorPositionLoop(), _Spool()

    _follow(loop, spool, lambda _: 2.0, seconds=3.0)

    assert math.isclose(spool.position, 2.0, abs_tol=0.05)


def test_a_one_sided_reference_does_not_wind_the_spool_in():
    """The bench failure: a never-negative reference must still bound travel.

    The reference here is a rectified sine -- positive for every tick of every
    stride, as the assist's is -- so commanding it as a velocity would integrate
    to metres of cable. Closed as a position loop it has to come back to zero
    between strides.
    """
    loop, spool = MotorPositionLoop(), _Spool()

    # 1 Hz, peaking at 5.5 rad: the shape and size of the 2026-09-21 run.
    _follow(loop, spool, lambda t: 5.5 * abs(math.sin(math.pi * t)), seconds=10.0)

    assert spool.position < 6.0, "the spool wound in instead of tracking"


def test_the_loop_is_silent_while_disabled():
    """A disabled loop commands zero however large the error it is shown."""
    loop = MotorPositionLoop()

    for _ in range(100):
        command = loop.step(
            reference_rad=5.0, measured_rad=0.0, enabled=False, time_difference=DT
        )
        assert command == 0.0


def test_enabling_resets_the_loop():
    """The model's enable port carries StatesWhenEnabling: reset.

    A session must therefore start from rest, whatever the previous one left
    behind -- not carry its integrator and damping filter into the first tick
    of the next.
    """
    loop, spool = MotorPositionLoop(), _Spool()
    _follow(loop, spool, lambda _: 5.0, seconds=2.0)

    loop.step(reference_rad=0.0, measured_rad=0.0, enabled=False, time_difference=DT)
    first = loop.step(
        reference_rad=0.0, measured_rad=0.0, enabled=True, time_difference=DT
    )

    # hip-controller's PID returns zero on its first call, having no previous
    # timestamp to take a step from -- which is exactly what a reset looks like.
    assert first == 0.0


def test_position_is_continuous_across_the_field_wrap():
    """A rollover must read as motion continuing, not as a 25 rad jump.

    The model accumulates the whole step it rejects, so the tick that wraps
    loses the real motion inside it -- under half a radian at the speed the
    command saturates to. What must not survive is the 25 rad discontinuity.
    """
    loop = MotorPositionLoop()
    wrap = FIELD_LIMIT_RAD

    before = loop.unwrap(wrap - 0.1)
    after = loop.unwrap(-wrap + 0.1)

    assert abs(after - before) < 0.5


def test_a_wrap_does_not_slam_the_command():
    """The reason unwrapping is here: a rollover must not become a full-speed command.

    Read raw, the error across the boundary steps by a full span, which this
    loop's gain would turn into hundreds of rad/s against a wearer's leg.
    """
    loop = MotorPositionLoop()
    wrap = FIELD_LIMIT_RAD

    before = loop.step(
        reference_rad=wrap, measured_rad=wrap - 0.1, enabled=True, time_difference=DT
    )
    after = loop.step(
        reference_rad=wrap, measured_rad=-wrap + 0.1, enabled=True, time_difference=DT
    )

    assert abs(after - before) < 5.0


def test_missing_feedback_holds_the_last_position():
    """NaN feedback must not poison the position with NaN."""
    loop = MotorPositionLoop()

    loop.unwrap(3.0)
    held = loop.unwrap(math.nan)

    assert held == 3.0
    command = loop.step(
        reference_rad=3.0, measured_rad=math.nan, enabled=True, time_difference=DT
    )
    assert not math.isnan(command)


def test_a_late_tick_cannot_rail_the_loop():
    """A stall hands the loop a huge step; the damping filter must not rail.

    The loop does not clamp its own output -- the rig saturates downstream,
    after the value has been fed back to the filter -- so what is asserted here
    is that a 5 s step produces the proportional term and nothing exploding on
    top of it.
    """
    loop = MotorPositionLoop()

    # The first enabled tick is hip-controller's own: with no previous
    # timestamp to take a step from it returns zero, so the stall lands on the
    # second.
    loop.step(reference_rad=1.0, measured_rad=0.0, enabled=True, time_difference=DT)
    command = loop.step(
        reference_rad=1.0, measured_rad=0.0, enabled=True, time_difference=5.0
    )

    assert math.isfinite(command)
    # kp * 1.0 rad of error, with the damping term a small correction on top.
    assert abs(command - CONFIG.proportional_gain) < 1.0


def test_the_loop_does_not_clamp_its_own_output():
    """Saturation belongs downstream, as it does in the model and on the rig.

    Clamping here would feed the damping filter a different signal from the
    rig's on every tick that saturates, because both store the unclipped value.
    """
    loop = MotorPositionLoop()

    loop.step(reference_rad=100.0, measured_rad=0.0, enabled=True, time_difference=DT)
    command = loop.step(
        reference_rad=100.0, measured_rad=0.0, enabled=True, time_difference=DT
    )

    assert command > 41.87, "the loop clamped instead of leaving it to the caller"


def test_reset_clears_the_accumulated_unwrap():
    """A new session starts from where the shaft is, not from the last one's count."""
    loop = MotorPositionLoop()
    wrap = FIELD_LIMIT_RAD

    loop.unwrap(wrap - 0.1)
    loop.unwrap(-wrap + 0.1)
    loop.reset()

    assert loop.unwrap(1.0) == 1.0
