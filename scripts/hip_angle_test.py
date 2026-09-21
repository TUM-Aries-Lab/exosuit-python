"""Read both hip angles live. No motors, no GPIO, no controller.

For settling whether the angle signal has the scale the amplitude stage
assumes. A 2026-09-21 walking run produced 0.53 / 0.65 rad of hip excursion
where the LocomotionMode rig produces 1.29 / 1.21 at a comparable cadence, and
a recording cannot say whether that is the walking or the signal.

Hold a known angle and compare. Standing upright is the zero; a thigh lifted to
horizontal is 90 degrees, which is the easiest pose to hit repeatably. Zeroing
on a keypress makes the reading a change from the pose you are in, so the
mounting offset does not matter.

The angle here is derived exactly as the control loop derives it -- the ``y``
component of the xyz Euler decomposition -- so what this prints is what the
controller sees, not a second opinion about it.
"""

import math
import time
import warnings

from imu_python.factory import IMUFactory
from loguru import logger

from exosuit_python.definitions import IMUConfig

# Slow enough to read while holding a pose, fast enough to watch it settle.
POLL_INTERVAL = 0.25

#: Poses worth checking, as (label, expected change from standing in degrees).
REFERENCE_POSES = (("thigh horizontal", 90.0), ("half of that", 45.0))


def detect_legs(config: IMUConfig):
    """Return the (left, right) IMU managers, matched as the exosuit matches them.

    :param config: The IMU configuration naming each leg's bus and descriptor.
    :return: ``(left, right)``, either of which may be None if unmatched.
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        managers = IMUFactory.detect_and_create(
            free_threading=True, log_data=False, create_mock=False
        )
    left = right = None
    for manager in managers:
        if (
            left is None
            and manager.i2c_id == config.left_leg_bus
            and manager.imu_descriptor == config.left_leg_descr
        ):
            left = manager
        elif (
            manager.i2c_id == config.right_leg_bus
            and manager.imu_descriptor == config.right_leg_descr
        ):
            right = manager
    return left, right


def read_angle(manager) -> float:
    """Return one limb's hip angle in radians, as the control loop derives it.

    :param manager: The IMU manager for that limb.
    :return: The angle in radians, or NaN if nothing could be read.
    :rtype: float
    """
    data = manager.get_data()
    if data is None:
        return math.nan
    return data.quat.to_euler(seq="xyz").y


def main() -> None:
    """Print both hip angles until interrupted, zeroing on a keypress."""
    left, right = detect_legs(IMUConfig())
    if left is None or right is None:
        logger.error(f"Could not match both legs. Left: {left} Right: {right}")
        return

    logger.info("Stand upright, then press Enter to zero.")
    for label, degrees in REFERENCE_POSES:
        logger.info(
            f"  {label:18s} {degrees:5.1f} deg = {math.radians(degrees):.2f} rad"
        )
    logger.info(
        "A held 90 deg reading about 0.8 rad rather than 1.57 means the angle is "
        "half scale, which is a signal problem and not a tuning one."
    )
    input("Enter to zero: ")

    zero_left, zero_right = read_angle(left), read_angle(right)
    logger.info(f"Zeroed at L {zero_left:+.3f} R {zero_right:+.3f} rad.")

    try:
        while True:
            angle_left = read_angle(left) - zero_left
            angle_right = read_angle(right) - zero_right
            logger.info(
                f"L {angle_left:+7.3f} rad ({math.degrees(angle_left):+6.1f} deg)"
                f"   R {angle_right:+7.3f} rad ({math.degrees(angle_right):+6.1f} deg)"
            )
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        logger.info("Done.")


if __name__ == "__main__":
    main()
