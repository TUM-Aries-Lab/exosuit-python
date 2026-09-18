"""Tests for the pre-tensioning chart ported from ``motor_control.py``."""

from motor_python.definitions import AK80_6_MOTOR_SPEC

from exosuit_python.definitions import TensionConfig, TensionState
from exosuit_python.tensioning import LegTensioner

DT = 0.01
CONFIG = TensionConfig()


def _run(tensioner: LegTensioner, torque_nm: float, on_off: int, seconds: float):
    """Step the tensioner for a while and return its last output."""
    output = (0.0, 0)
    for _ in range(int(seconds / DT)):
        output = tensioner.step(torque_nm, on_off, DT)
    return output


def test_starts_in_reset_and_stays_put_until_the_switch_is_held():
    """An untouched tensioner must not command any motion."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    assert tensioner.state is TensionState.RESET

    velocity, enable = _run(tensioner, torque_nm=0.0, on_off=0, seconds=1.0)

    assert tensioner.state is TensionState.RESET
    assert velocity == 0.0
    assert enable == 0


def test_holding_the_switch_pulls_at_the_leg_velocity():
    """RESET -> TENSIONING on the switch, commanding that leg's velocity."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)

    velocity, enable = tensioner.step(0.0, on_off=1, time_difference=DT)

    assert tensioner.state is TensionState.TENSIONING
    assert velocity == CONFIG.tensioning_velocity_left_rad_per_sec
    assert enable == 1


def test_the_legs_pull_in_opposite_directions():
    """The legs are mirrored, so their velocity commands differ in sign."""
    left = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    right = LegTensioner(CONFIG.tensioning_velocity_right_rad_per_sec)

    left_velocity, _ = left.step(0.0, on_off=1, time_difference=DT)
    right_velocity, _ = right.step(0.0, on_off=1, time_difference=DT)

    assert left_velocity > 0
    assert right_velocity < 0


def test_releasing_the_switch_aborts_without_waiting_for_the_threshold():
    """TENSIONING -> DISABLE on release, so the wearer can always abort."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)
    assert tensioner.state is TensionState.TENSIONING

    tensioner.step(0.0, on_off=0, time_difference=DT)

    assert tensioner.state is TensionState.DISABLE
    assert tensioner.finished


def test_crossing_the_threshold_stops_the_pull_then_disables_after_the_hold():
    """TENSIONING -> STOP on torque, STOP -> DISABLE after stop_hold_time."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)

    # Well above the threshold, but the filter needs time to catch up.
    _run(tensioner, torque_nm=5.0, on_off=1, seconds=1.0)
    assert tensioner.state is TensionState.STOP
    assert tensioner.velocity_rad_per_sec == 0.0
    # Enable is deliberately untouched by STOP, so the motor still holds.
    assert tensioner.enable == 1

    _run(tensioner, torque_nm=5.0, on_off=1, seconds=CONFIG.stop_hold_time + 0.1)

    assert tensioner.state is TensionState.DISABLE
    assert tensioner.enable == -1


def test_torque_below_the_threshold_keeps_pulling():
    """A load under the threshold must not end the pull."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)

    below = CONFIG.torque_threshold_nm * 0.5
    velocity, enable = _run(tensioner, torque_nm=below, on_off=1, seconds=3.0)

    assert tensioner.state is TensionState.TENSIONING
    assert velocity == CONFIG.tensioning_velocity_left_rad_per_sec
    assert enable == 1


def test_the_filter_rejects_the_largest_single_spike_the_wire_can_carry():
    """The LPF exists so a transient cannot end tensioning on its own.

    The MIT feedback field is decoded against the motor profile's torque
    range, so AK80_6_MIT_LIMITS.t_max is the largest value that can physically
    arrive in one frame. One sample of it must not trip the threshold --
    otherwise a single corrupt frame ends the pull early.
    """
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)

    tensioner.step(
        AK80_6_MOTOR_SPEC.mit_mode_limits.t_max, on_off=1, time_difference=DT
    )

    assert tensioner.state is TensionState.TENSIONING


def test_negative_torque_counts_the_same_as_positive():
    """The chart thresholds on |LPF(torque)|, so the sign cannot matter."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_right_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)

    _run(tensioner, torque_nm=-5.0, on_off=1, seconds=1.0)

    assert tensioner.state is TensionState.STOP


def test_reset_returns_a_finished_tensioner_for_reuse():
    """A disabled leg must come back clean for the next session."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)
    tensioner.step(0.0, on_off=0, time_difference=DT)
    assert tensioner.finished

    tensioner.reset()

    assert tensioner.state is TensionState.RESET
    assert not tensioner.finished
    assert tensioner.velocity_rad_per_sec == 0.0
    assert tensioner.enable == 0


def test_the_disable_is_reported_once_not_every_tick():
    """Disabling is an edge, so a caller can act on it exactly once.

    The CAN ``stop()`` blocks for 60 ms of settling -- six ticks at 100 Hz --
    so calling it every lap while the other leg is still pulling would stall
    the loop and hand the running leg a dt far larger than it was stepped
    with. The rig fires its enable/disable frames on transitions for the same
    reason.
    """
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    # Enabling is an edge too: enable goes 0 -> 1 on this tick.
    tensioner.step(0.0, on_off=1, time_difference=DT)
    assert tensioner.enable_changed

    tensioner.step(0.0, on_off=0, time_difference=DT)
    assert tensioner.finished
    assert tensioner.enable_changed

    for _ in range(20):
        tensioner.step(0.0, on_off=0, time_difference=DT)
        assert not tensioner.enable_changed


def test_reaching_the_threshold_also_reports_its_disable_once():
    """The same holds for the STOP -> DISABLE path."""
    tensioner = LegTensioner(CONFIG.tensioning_velocity_left_rad_per_sec)
    tensioner.step(0.0, on_off=1, time_difference=DT)
    _run(tensioner, torque_nm=5.0, on_off=1, seconds=1.0)
    assert tensioner.state is TensionState.STOP

    transitions = 0
    for _ in range(int((CONFIG.stop_hold_time + 0.5) / DT)):
        tensioner.step(5.0, on_off=1, time_difference=DT)
        if tensioner.enable_changed:
            transitions += 1

    assert tensioner.state is TensionState.DISABLE
    assert transitions == 1
