"""Sample doc string."""

import argparse

from exosuit_python.definitions import DEFAULT_LOG_LEVEL, LogLevel
from exosuit_python.exosuit import Exosuit, ExosuitConfig
from exosuit_python.utils import setup_logger


def main(log_level: str, stderr_level: str, mock: bool) -> None:  # pragma: no cover
    """Run the main pipeline.

    :param log_level: The log level to use.
    :param stderr_level: The std err level to use.
    :return: None
    """
    setup_logger(log_level=log_level, stderr_level=stderr_level)

    config = ExosuitConfig(frequency=100, mock=mock)
    Exosuit(config=config)

    # Current implementation uses main thread for exosuit state handler loop


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
        default=False,
        help="Use mock devices.",
        required=False,
        type=bool,
    )
    args = parser.parse_args()

    main(log_level=args.log_level, stderr_level=args.stderr_level, mock=args.mock)
