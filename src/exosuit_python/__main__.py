"""Sample doc string."""

import argparse
import time

from loguru import logger

from exosuit_python.definitions import (
    DEFAULT_LOG_LEVEL,
    POWER_SWITCH,
    TENSION_SWITCH,
    LogLevel,
)
from exosuit_python.exosuit import Exosuit, ExosuitConfig, ExosuitStates
from exosuit_python.gpio import MockGPIO
from exosuit_python.utils import setup_logger


def main(
    log_level: str, stderr_level: str, mock_devices: bool, test_gpio: bool
) -> None:  # pragma: no cover
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    config = ExosuitConfig(
        frequency=100, mock_devices=mock_devices, test_gpio=test_gpio
    )
    exosuit = Exosuit(config=config)
    try:
        while True:
            test_pipeline(exosuit)
    except KeyboardInterrupt:
        exosuit._cleanup()


def test_pipeline(exosuit: Exosuit) -> None:
    """Run the test pipeline."""
    if isinstance(exosuit.gpio, MockGPIO):
        # wait for initialization
        time.sleep(2)
        # activate pre-tensioning
        logger.info("Simulating tensioning switch ON...")
        exosuit.gpio.simulate_switch(TENSION_SWITCH, exosuit.on_signal)
        time.sleep(1)
        # deactivate pre-tensioning
        logger.info("Simulating tensioning switch OFF...")
        exosuit.gpio.simulate_switch(TENSION_SWITCH, exosuit.off_signal)
        time.sleep(3)
        # power on
        logger.info("Simulating power switch ON...")
        exosuit.gpio.simulate_switch(POWER_SWITCH, exosuit.on_signal)
        time.sleep(10)
        # power off
        logger.info("Simulating power switch OFF...")
        exosuit.gpio.simulate_switch(POWER_SWITCH, exosuit.off_signal)
        time.sleep(4)
    else:
        if exosuit._status not in [ExosuitStates.INITIALIZING, ExosuitStates.STOPPED]:
            power_state = exosuit.gpio.input(POWER_SWITCH)
            logger.info(power_state)
        time.sleep(0.2)


if __name__ == "__main__":  # pragma: no cover
    parser = argparse.ArgumentParser("Run the pipeline.")
    parser.add_argument(
        "--log-level",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the log level.",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--stderr-level",
        default=DEFAULT_LOG_LEVEL,
        choices=list(LogLevel()),
        help="Set the std err level.",
        required=False,
        type=str,
    )
    parser.add_argument(
        "--mock",
        help="Use mock devices.",
        action="store_true",
    )
    parser.add_argument(
        "--gpio",
        help="Test GPIO on the Jetson.",
        action="store_true",
    )
    args = parser.parse_args()

    main(
        log_level=args.log_level,
        stderr_level=args.stderr_level,
        mock_devices=args.mock,
        test_gpio=args.gpio,
    )
