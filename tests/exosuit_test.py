"""Test the main program."""

from exosuit_python.exosuit import Exosuit, ExosuitConfig


def test_exosuit():
    """Test the main function."""
    # Arrange
    config = ExosuitConfig(frequency=100)

    # Act
    exosuit = Exosuit(config)
    exosuit.run()

    # Assert
    assert not exosuit._is_running
