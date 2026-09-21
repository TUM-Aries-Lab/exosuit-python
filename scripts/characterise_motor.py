"""Measure what a motor actually does with a velocity command.

The MIT law is ``tau = t_ff + kp*(p_des - p) + kd*(v_des - v)``. With kp and
t_ff at zero, as the assist runs it, a commanded velocity against a stationary
shaft should produce ``kd * v_des`` of torque and the shaft should turn.

On 2026-09-21 it did not. From t=8.33 to 8.42 the right motor was commanded up
to -4.6 rad/s at kd=1.0 and reported 0.64 N*m with the shaft stationary, where
the law predicts 4.6 N*m. It then broke loose in a single tick to -11.05 N*m
and slewed to -30.8 rad/s in about 50 ms, and 0.41 s later it dropped out of
MIT mode altogether.

Three explanations for that run have already been tested against the recordings
and failed -- the amplitude gate, an overcurrent from friction, and steps in
the reference. This script stops inferring and measures the plant: a staircase
of commanded velocities, one motor at a time, with nothing else in the way.

Run it with the **tendon disconnected** first. That is the clean measurement:
the only load is the motor's own rotor and whatever friction the spool carries.
Repeat with the tendon attached to see what the suit adds. Comparing left
against right in the same condition is the point -- their peak torques have
differed by a factor of 2.6 across two runs.

    uv run python scripts/characterise_motor.py --side right
    uv run python scripts/characterise_motor.py --side right --kd 2.0

The CSV it writes is the deliverable; the printed summary is a sanity check.
"""

import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

from loguru import logger
from motor_python import create_can_motor

from exosuit_python.definitions import MOTOR_CAN_ID_LEFT, MOTOR_CAN_ID_RIGHT

#: Commanded velocities in rad/s at the output shaft. Small ones first, so a
#: misbehaving motor shows itself before the script asks for anything fast.
STAIRCASE_RAD_PER_SEC = (1.0, 2.0, 5.0, 10.0, 20.0, 30.0)
#: Seconds held at each step, and the rest between them.
STEP_SECONDS = 1.5
REST_SECONDS = 1.0
#: Sampling rate for the log, matched to the control loop's.
SAMPLE_HZ = 100.0
#: Ignored at the start of each step, so the average describes the steady state
#: rather than the acceleration into it.
SETTLE_FRACTION = 0.4

OUTPUT_DIR = Path("data/characterisation")


def read_state(motor) -> tuple[float, float, float]:
    """Return (position_rad, velocity_rad_per_sec, torque_nm) from the cache.

    Read from the transport's cached feedback rather than requested: a status
    request is byte-identical to the MIT enable frame, so polling would drive
    the motor being measured.

    :param motor: The motor to read.
    :return: Position, velocity and torque, NaN where unavailable.
    :rtype: tuple[float, float, float]
    """
    feedback = getattr(motor, "_last_feedback", None)
    if feedback is None:
        return math.nan, math.nan, math.nan
    to_rad_s = getattr(motor, "_erpm_to_rad_s", None)
    speed = float(getattr(feedback, "speed_erpm", math.nan))
    return (
        math.radians(float(getattr(feedback, "position_degrees", math.nan))),
        to_rad_s(speed) if to_rad_s is not None else math.nan,
        float(getattr(feedback, "current_amps", math.nan)),
    )


def run_step(motor, command: float, kd: float, rows: list, step_index: int) -> dict:
    """Hold one commanded velocity and record what the motor does.

    :param motor: The motor under test.
    :param command: Commanded velocity in rad/s at the output shaft.
    :param kd: The MIT damping gain to send with it.
    :param rows: Sample list to append to, shared across steps.
    :param step_index: Which step of the staircase this is.
    :return: A summary of the step's steady state.
    :rtype: dict
    """
    started = time.monotonic()
    settled_after = STEP_SECONDS * SETTLE_FRACTION
    steady_speed: list[float] = []
    steady_torque: list[float] = []

    while True:
        elapsed = time.monotonic() - started
        if elapsed >= STEP_SECONDS:
            break
        motor.set_mit_mode(
            pos_rad=0.0, vel_rad_s=command, kp=0.0, kd=kd, torque_ff_nm=0.0
        )
        position, speed, torque = read_state(motor)
        rows.append(
            {
                "step_index": step_index,
                "elapsed_s": round(elapsed, 4),
                "command_rad_per_sec": command,
                "kd": kd,
                "position_rad": position,
                "speed_rad_per_sec": speed,
                "torque_nm": torque,
            }
        )
        if elapsed >= settled_after and not math.isnan(speed):
            steady_speed.append(speed)
            steady_torque.append(torque)
        time.sleep(1.0 / SAMPLE_HZ)

    mean_speed = sum(steady_speed) / len(steady_speed) if steady_speed else math.nan
    mean_torque = sum(steady_torque) / len(steady_torque) if steady_torque else math.nan
    velocity_error = command - mean_speed
    return {
        "command": command,
        "speed": mean_speed,
        "torque": mean_torque,
        "error": velocity_error,
        # What kd the motor is behaving as if it had. The commanded value is
        # the whole question: torque should be kd times the velocity error.
        "implied_kd": (
            mean_torque / velocity_error if abs(velocity_error) > 0.05 else math.nan
        ),
        "tracked": (mean_speed / command) if command else math.nan,
    }


def rest(motor, kd: float) -> None:
    """Command zero for the rest period, keeping MIT mode alive.

    ``stop()`` is deliberately not used between steps: on this transport it
    disables MIT mode and blocks for the best part of a tenth of a second.

    :param motor: The motor under test.
    :param kd: The damping gain to hold it with.
    :return: None
    """
    until = time.monotonic() + REST_SECONDS
    while time.monotonic() < until:
        motor.set_mit_mode(pos_rad=0.0, vel_rad_s=0.0, kp=0.0, kd=kd, torque_ff_nm=0.0)
        time.sleep(1.0 / SAMPLE_HZ)


def write_csv(rows: list, side: str, kd: float) -> Path:
    """Write the samples and return where they went.

    :param rows: Every sample taken across the staircase.
    :param side: Which leg was measured.
    :param kd: The damping gain used.
    :return: The file written.
    :rtype: Path
    """
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    path = OUTPUT_DIR / f"characterisation_{side}_kd{kd:g}_{stamp}.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return path


def main() -> None:
    """Run the staircase on one motor and report what it did."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--side", choices=("left", "right"), required=True, help="Which leg to measure."
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=1.0,
        help="MIT damping gain. 1.0 is what the assist runs.",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Command the staircase negative instead of positive.",
    )
    parser.add_argument(
        "--yes", action="store_true", help="Skip the confirmation prompt."
    )
    args = parser.parse_args()

    can_id = MOTOR_CAN_ID_LEFT if args.side == "left" else MOTOR_CAN_ID_RIGHT
    sign = -1.0 if args.reverse else 1.0
    steps = [sign * v for v in STAIRCASE_RAD_PER_SEC]

    logger.warning(
        f"About to spin the {args.side} motor (CAN id {can_id}) up to "
        f"{abs(steps[-1]):.0f} rad/s at kd={args.kd:g}. "
        f"Disconnect the tendon for the clean measurement, and keep clear."
    )
    if not args.yes:
        input("Enter to start, Ctrl-C to abort: ")

    motor = create_can_motor("AK80-6", motor_can_id=can_id)
    rows: list[dict] = []
    summaries: list[dict] = []
    try:
        motor.enable_mit_mode()
        for index, command in enumerate(steps):
            logger.info(f"Step {index + 1}/{len(steps)}: {command:+.1f} rad/s")
            summaries.append(run_step(motor, command, args.kd, rows, index))
            rest(motor, args.kd)
    except KeyboardInterrupt:
        logger.warning("Aborted.")
    finally:
        motor.stop()
        motor.close()

    if not rows:
        logger.error("No samples recorded.")
        return

    path = write_csv(rows, args.side, args.kd)
    logger.info(f"Wrote {len(rows)} samples to '{path}'.")

    logger.info(
        f"{'cmd':>8} {'reached':>9} {'tracked':>8} {'v error':>9} "
        f"{'torque':>8} {'implied kd':>11}"
    )
    for summary in summaries:
        logger.info(
            f"{summary['command']:8.1f} {summary['speed']:9.2f}"
            f" {100 * summary['tracked']:7.0f}% {summary['error']:9.2f}"
            f" {summary['torque']:8.2f} {summary['implied_kd']:11.2f}"
        )
    logger.info(
        f"Commanded kd was {args.kd:g}. Implied kd well below it means the motor "
        f"is not producing the torque its command calls for, which is the "
        f"anomaly this script exists to confirm or rule out."
    )


if __name__ == "__main__":
    main()
