"""Test the utils module."""

from pathlib import Path
from tempfile import TemporaryDirectory

from loguru import logger

from exosuit_python.definitions import DEFAULT_LOG_FILENAME, LogLevel
from exosuit_python.utils import setup_logger


def test_logger_init() -> None:
    """Test logger initialization."""
    with TemporaryDirectory() as log_dir:
        log_dir_path = Path(log_dir)
        log_filepath = setup_logger(filename=DEFAULT_LOG_FILENAME, log_dir=log_dir_path)
        assert Path(log_filepath).exists()
        # Release the sink before the directory is torn down. setup_logger
        # adds the file with enqueue=True, so loguru holds the handle open in
        # a writer thread. On Windows a directory containing an open file
        # cannot be deleted, so TemporaryDirectory raises WinError 32 on exit;
        # POSIX unlink() tolerates an open file, which is why this only bites
        # locally and never in the Linux CI.
        logger.remove()
    assert not Path(log_filepath).exists()


def test_log_level() -> None:
    """Test the log level."""
    # Act
    log_levels = list(LogLevel())

    # Assert
    assert type(log_levels) is list
