"""Configure the logger."""

import sys
from datetime import datetime
from pathlib import Path

from loguru import logger

from exosuit_python.definitions import (
    DATE_FORMAT,
    DEFAULT_LOG_FILENAME,
    DEFAULT_LOG_LEVEL,
    ENCODING,
    LOG_DIR,
)


def create_timestamped_filepath(suffix: str, output_dir: Path, prefix: str) -> Path:
    """Generate a timestamped filename.

    :param suffix: Suffix to append to the timestamped filename.
    :param output_dir: Output directory.
    :param prefix: Prefix to append to the timestamped filename.
    :return: Path to the timestamped filename.
    """
    timestamp = datetime.now().strftime(DATE_FORMAT)
    filepath = output_dir / f"{prefix}_{timestamp}.{suffix}"
    return filepath


def setup_logger(
    filename: str = DEFAULT_LOG_FILENAME,
    stderr_level: str = DEFAULT_LOG_LEVEL,
    log_level: str = DEFAULT_LOG_LEVEL,
    log_dir: Path = LOG_DIR,
) -> Path:
    """Configure the logger.

    :param filename: Name of the file to create.
    :param stderr_level: Logging level to use.
    :param log_level: Logging level to use.
    :param log_dir: Logging directory to use.
    :return: Path to the created logfile.
    """
    logger.remove()

    filepath = create_timestamped_filepath(
        output_dir=log_dir, prefix=filename, suffix="log"
    )
    filepath.parent.mkdir(parents=True, exist_ok=True)  # create dirs if missing
    filepath.touch(exist_ok=True)  # create empty file (don't overwrite)
    logger.add(sys.stderr, level=stderr_level)
    logger.add(filepath, level=log_level, encoding=ENCODING, enqueue=True)
    logger.info(f"Logging to '{filepath}'.")
    return filepath
