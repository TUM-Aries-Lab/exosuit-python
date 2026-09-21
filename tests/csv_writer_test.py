"""Test the CSV writer for recording exosuit data."""

import tempfile
from pathlib import Path

import pandas as pd
import pytest

from exosuit_python.csv_writer import (
    CSVWriter,
    RecordData,
    RecordDataColumnNames,
    SensorSignal,
)


class TestCSVWriterInit:
    """Test CSVWriter initialization."""

    def test_init_creates_empty_rows(self) -> None:
        """Test that initialization creates an empty rows list."""
        writer = CSVWriter()
        assert writer.rows == []
        assert isinstance(writer.rows, list)


class TestCSVWriterReset:
    """Test CSVWriter reset functionality."""

    def test_reset_clears_rows(self) -> None:
        """Test that reset clears the rows list."""
        writer = CSVWriter()
        # Add some data
        data = RecordData(
            timestamp=1.0,
            raw_signal_left=SensorSignal(
                timestamp=1.0, angle_rad=0.1, velocity_rad_per_sec=0.2
            ),
            filtered_signal_left=SensorSignal(
                timestamp=1.0, angle_rad=0.15, velocity_rad_per_sec=0.25
            ),
            raw_signal_right=SensorSignal(
                timestamp=1.0, angle_rad=0.3, velocity_rad_per_sec=0.4
            ),
            filtered_signal_right=SensorSignal(
                timestamp=1.0, angle_rad=0.35, velocity_rad_per_sec=0.45
            ),
            motor_torque_nm_left=1.5,
            motor_speed_rad_per_sec_left=2.0,
            motor_position_rad_left=0.5,
            motor_torque_nm_right=1.6,
            motor_speed_rad_per_sec_right=2.1,
            motor_position_rad_right=0.6,
        )
        writer.append_data(data)
        assert len(writer.rows) == 1

        # Reset
        writer.reset()
        assert writer.rows == []


class TestCSVWriterAppendData:
    """Test CSVWriter append_data functionality."""

    @pytest.mark.parametrize(
        "record_data",
        [
            # Basic zero values
            RecordData(
                timestamp=0.0,
                raw_signal_left=SensorSignal(
                    timestamp=0.0, angle_rad=0.0, velocity_rad_per_sec=0.0
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=0.0, angle_rad=0.0, velocity_rad_per_sec=0.0
                ),
                raw_signal_right=SensorSignal(
                    timestamp=0.0, angle_rad=0.0, velocity_rad_per_sec=0.0
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=0.0, angle_rad=0.0, velocity_rad_per_sec=0.0
                ),
                motor_torque_nm_left=0.0,
                motor_speed_rad_per_sec_left=0.0,
                motor_position_rad_left=0.0,
                motor_torque_nm_right=0.0,
                motor_speed_rad_per_sec_right=0.0,
                motor_position_rad_right=0.0,
            ),
            # Small positive values
            RecordData(
                timestamp=1.5,
                raw_signal_left=SensorSignal(
                    timestamp=1.5, angle_rad=0.1, velocity_rad_per_sec=0.2
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=1.5, angle_rad=0.15, velocity_rad_per_sec=0.25
                ),
                raw_signal_right=SensorSignal(
                    timestamp=1.5, angle_rad=0.3, velocity_rad_per_sec=0.4
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=1.5, angle_rad=0.35, velocity_rad_per_sec=0.45
                ),
                motor_torque_nm_left=1.5,
                motor_speed_rad_per_sec_left=2.0,
                motor_position_rad_left=0.5,
                motor_torque_nm_right=1.6,
                motor_speed_rad_per_sec_right=2.1,
                motor_position_rad_right=0.6,
            ),
            # Large values
            RecordData(
                timestamp=1000.5,
                raw_signal_left=SensorSignal(
                    timestamp=1000.5, angle_rad=3.14, velocity_rad_per_sec=6.28
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=1000.5, angle_rad=2.71, velocity_rad_per_sec=5.42
                ),
                raw_signal_right=SensorSignal(
                    timestamp=1000.5, angle_rad=1.41, velocity_rad_per_sec=2.82
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=1000.5, angle_rad=1.73, velocity_rad_per_sec=3.46
                ),
                motor_torque_nm_left=100.0,
                motor_speed_rad_per_sec_left=200.0,
                motor_position_rad_left=50.0,
                motor_torque_nm_right=101.0,
                motor_speed_rad_per_sec_right=201.0,
                motor_position_rad_right=51.0,
            ),
            # Negative values
            RecordData(
                timestamp=2.0,
                raw_signal_left=SensorSignal(
                    timestamp=2.0, angle_rad=-0.5, velocity_rad_per_sec=-1.0
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=2.0, angle_rad=-0.4, velocity_rad_per_sec=-0.9
                ),
                raw_signal_right=SensorSignal(
                    timestamp=2.0, angle_rad=-0.2, velocity_rad_per_sec=-0.3
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=2.0, angle_rad=-0.15, velocity_rad_per_sec=-0.25
                ),
                motor_torque_nm_left=-2.0,
                motor_speed_rad_per_sec_left=-3.0,
                motor_position_rad_left=-1.0,
                motor_torque_nm_right=-2.1,
                motor_speed_rad_per_sec_right=-3.1,
                motor_position_rad_right=-1.1,
            ),
            # Mixed sign values
            RecordData(
                timestamp=5.5,
                raw_signal_left=SensorSignal(
                    timestamp=5.5, angle_rad=-0.82, velocity_rad_per_sec=-7.3
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=5.5, angle_rad=0.14, velocity_rad_per_sec=3.78
                ),
                raw_signal_right=SensorSignal(
                    timestamp=5.5, angle_rad=0.21, velocity_rad_per_sec=2.9
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=5.5, angle_rad=-0.12, velocity_rad_per_sec=5.1
                ),
                motor_torque_nm_left=2.3,
                motor_speed_rad_per_sec_left=-1.0,
                motor_position_rad_left=0.0,
                motor_torque_nm_right=2.4,
                motor_speed_rad_per_sec_right=1.0,
                motor_position_rad_right=3.3,
            ),
            # Very small values
            RecordData(
                timestamp=0.001,
                raw_signal_left=SensorSignal(
                    timestamp=0.001, angle_rad=0.0001, velocity_rad_per_sec=0.0002
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=0.001, angle_rad=0.00015, velocity_rad_per_sec=0.00025
                ),
                raw_signal_right=SensorSignal(
                    timestamp=0.001, angle_rad=0.0003, velocity_rad_per_sec=0.0004
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=0.001, angle_rad=0.00035, velocity_rad_per_sec=0.00045
                ),
                motor_torque_nm_left=0.0015,
                motor_speed_rad_per_sec_left=0.002,
                motor_position_rad_left=0.0005,
                motor_torque_nm_right=0.0016,
                motor_speed_rad_per_sec_right=0.0021,
                motor_position_rad_right=0.0006,
            ),
        ],
    )
    def test_append_data_various_values(self, record_data: RecordData) -> None:
        """Test appending various data values."""
        writer = CSVWriter()
        writer.append_data(record_data)

        assert len(writer.rows) == 1
        row = writer.rows[0]
        assert row[RecordDataColumnNames.TIMESTAMP.value] == record_data.timestamp
        assert (
            row[RecordDataColumnNames.RAW_ANGLE_LEFT.value]
            == record_data.raw_signal_left.angle_rad
        )
        assert (
            row[RecordDataColumnNames.RAW_VELOCITY_LEFT.value]
            == record_data.raw_signal_left.velocity_rad_per_sec
        )
        assert (
            row[RecordDataColumnNames.FILTERED_ANGLE_LEFT.value]
            == record_data.filtered_signal_left.angle_rad
        )
        assert (
            row[RecordDataColumnNames.FILTERED_VELOCITY_LEFT.value]
            == record_data.filtered_signal_left.velocity_rad_per_sec
        )
        assert (
            row[RecordDataColumnNames.RAW_ANGLE_RIGHT.value]
            == record_data.raw_signal_right.angle_rad
        )
        assert (
            row[RecordDataColumnNames.RAW_VELOCITY_RIGHT.value]
            == record_data.raw_signal_right.velocity_rad_per_sec
        )
        assert (
            row[RecordDataColumnNames.FILTERED_ANGLE_RIGHT.value]
            == record_data.filtered_signal_right.angle_rad
        )
        assert (
            row[RecordDataColumnNames.FILTERED_VELOCITY_RIGHT.value]
            == record_data.filtered_signal_right.velocity_rad_per_sec
        )
        assert (
            row[RecordDataColumnNames.MOTOR_TORQUE_NM_LEFT.value]
            == record_data.motor_torque_nm_left
        )
        assert (
            row[RecordDataColumnNames.MOTOR_SPEED_RAD_PER_SEC_LEFT.value]
            == record_data.motor_speed_rad_per_sec_left
        )
        assert (
            row[RecordDataColumnNames.MOTOR_POSITION_RAD_LEFT.value]
            == record_data.motor_position_rad_left
        )
        assert (
            row[RecordDataColumnNames.MOTOR_TORQUE_NM_RIGHT.value]
            == record_data.motor_torque_nm_right
        )
        assert (
            row[RecordDataColumnNames.MOTOR_SPEED_RAD_PER_SEC_RIGHT.value]
            == record_data.motor_speed_rad_per_sec_right
        )
        assert (
            row[RecordDataColumnNames.MOTOR_POSITION_RAD_RIGHT.value]
            == record_data.motor_position_rad_right
        )

    @pytest.mark.parametrize("num_appends", [1, 2, 5, 10])
    def test_append_data_multiple_times(self, num_appends) -> None:
        """Test appending data multiple times."""
        writer = CSVWriter()
        data_list = []
        for i in range(num_appends):
            data = RecordData(
                timestamp=float(i),
                raw_signal_left=SensorSignal(
                    timestamp=float(i), angle_rad=0.1 * i, velocity_rad_per_sec=0.2 * i
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=float(i),
                    angle_rad=0.15 * i,
                    velocity_rad_per_sec=0.25 * i,
                ),
                raw_signal_right=SensorSignal(
                    timestamp=float(i), angle_rad=0.3 * i, velocity_rad_per_sec=0.4 * i
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=float(i),
                    angle_rad=0.35 * i,
                    velocity_rad_per_sec=0.45 * i,
                ),
                motor_torque_nm_left=1.5 * i,
                motor_speed_rad_per_sec_left=2.0 * i,
                motor_position_rad_left=0.5 * i,
                motor_torque_nm_right=1.6 * i,
                motor_speed_rad_per_sec_right=2.1 * i,
                motor_position_rad_right=0.6 * i,
            )
            data_list.append(data)
            writer.append_data(data)

        assert len(writer.rows) == num_appends
        for i, expected_data in enumerate(data_list):
            row = writer.rows[i]
            assert row[RecordDataColumnNames.TIMESTAMP.value] == expected_data.timestamp
            assert (
                row[RecordDataColumnNames.RAW_ANGLE_LEFT.value]
                == expected_data.raw_signal_left.angle_rad
            )


class TestCSVWriterSaveData:
    """Test CSVWriter save_data functionality."""

    @pytest.mark.parametrize(
        "record_data",
        [
            RecordData(
                timestamp=1.0,
                raw_signal_left=SensorSignal(
                    timestamp=1.0, angle_rad=0.1, velocity_rad_per_sec=0.2
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=1.0, angle_rad=0.15, velocity_rad_per_sec=0.25
                ),
                raw_signal_right=SensorSignal(
                    timestamp=1.0, angle_rad=0.3, velocity_rad_per_sec=0.4
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=1.0, angle_rad=0.35, velocity_rad_per_sec=0.45
                ),
                motor_torque_nm_left=1.5,
                motor_speed_rad_per_sec_left=2.0,
                motor_position_rad_left=0.5,
                motor_torque_nm_right=1.6,
                motor_speed_rad_per_sec_right=2.1,
                motor_position_rad_right=0.6,
            ),
            RecordData(
                timestamp=2.5,
                raw_signal_left=SensorSignal(
                    timestamp=2.5, angle_rad=-0.5, velocity_rad_per_sec=-1.0
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=2.5, angle_rad=-0.4, velocity_rad_per_sec=-0.9
                ),
                raw_signal_right=SensorSignal(
                    timestamp=2.5, angle_rad=-0.2, velocity_rad_per_sec=-0.3
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=2.5, angle_rad=-0.15, velocity_rad_per_sec=-0.25
                ),
                motor_torque_nm_left=-2.0,
                motor_speed_rad_per_sec_left=-3.0,
                motor_position_rad_left=-1.0,
                motor_torque_nm_right=-2.1,
                motor_speed_rad_per_sec_right=-3.1,
                motor_position_rad_right=-1.1,
            ),
        ],
    )
    def test_save_data_creates_file(self, record_data: RecordData) -> None:
        """Test that save_data creates a valid CSV file."""
        writer = CSVWriter()
        writer.append_data(record_data)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = writer.save_data(output_dir=Path(tmpdir))

            assert filepath.exists()
            assert filepath.suffix == ".csv"
            assert "recording_file" in filepath.name

            # Read and verify CSV content
            df = pd.read_csv(filepath)
            assert len(df) == 1
            assert (
                df[RecordDataColumnNames.TIMESTAMP.value].iloc[0]
                == record_data.timestamp
            )

    def test_save_data_multiple_rows(self) -> None:
        """Test saving data with multiple rows."""
        writer = CSVWriter()
        for i in range(3):
            data = RecordData(
                timestamp=float(i),
                raw_signal_left=SensorSignal(
                    timestamp=float(i), angle_rad=0.1 * i, velocity_rad_per_sec=0.2 * i
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=float(i),
                    angle_rad=0.15 * i,
                    velocity_rad_per_sec=0.25 * i,
                ),
                raw_signal_right=SensorSignal(
                    timestamp=float(i), angle_rad=0.3 * i, velocity_rad_per_sec=0.4 * i
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=float(i),
                    angle_rad=0.35 * i,
                    velocity_rad_per_sec=0.45 * i,
                ),
                motor_torque_nm_left=1.5 * i,
                motor_speed_rad_per_sec_left=2.0 * i,
                motor_position_rad_left=0.5 * i,
                motor_torque_nm_right=1.6 * i,
                motor_speed_rad_per_sec_right=2.1 * i,
                motor_position_rad_right=0.6 * i,
            )
            writer.append_data(data)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = writer.save_data(output_dir=Path(tmpdir))
            df = pd.read_csv(filepath)
            assert len(df) == 3

    def test_save_data_preserves_all_columns(self) -> None:
        """Test that all columns are preserved in the saved CSV."""
        writer = CSVWriter()
        data = RecordData(
            timestamp=1.0,
            raw_signal_left=SensorSignal(
                timestamp=1.0, angle_rad=0.1, velocity_rad_per_sec=0.2
            ),
            filtered_signal_left=SensorSignal(
                timestamp=1.0, angle_rad=0.15, velocity_rad_per_sec=0.25
            ),
            raw_signal_right=SensorSignal(
                timestamp=1.0, angle_rad=0.3, velocity_rad_per_sec=0.4
            ),
            filtered_signal_right=SensorSignal(
                timestamp=1.0, angle_rad=0.35, velocity_rad_per_sec=0.45
            ),
            motor_torque_nm_left=1.5,
            motor_speed_rad_per_sec_left=2.0,
            motor_position_rad_left=0.5,
            motor_torque_nm_right=1.6,
            motor_speed_rad_per_sec_right=2.1,
            motor_position_rad_right=0.6,
        )
        writer.append_data(data)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = writer.save_data(output_dir=Path(tmpdir))
            df = pd.read_csv(filepath)

            expected_columns = [col.value for col in RecordDataColumnNames]
            assert list(df.columns) == expected_columns

    @pytest.mark.parametrize("num_rows", [1, 5, 10, 100])
    def test_save_data_with_varying_row_counts(self, num_rows) -> None:
        """Test saving data with varying number of rows."""
        writer = CSVWriter()
        for i in range(num_rows):
            data = RecordData(
                timestamp=float(i),
                raw_signal_left=SensorSignal(
                    timestamp=float(i), angle_rad=0.1, velocity_rad_per_sec=0.2
                ),
                filtered_signal_left=SensorSignal(
                    timestamp=float(i), angle_rad=0.15, velocity_rad_per_sec=0.25
                ),
                raw_signal_right=SensorSignal(
                    timestamp=float(i), angle_rad=0.3, velocity_rad_per_sec=0.4
                ),
                filtered_signal_right=SensorSignal(
                    timestamp=float(i), angle_rad=0.35, velocity_rad_per_sec=0.45
                ),
                motor_torque_nm_left=1.5,
                motor_speed_rad_per_sec_left=2.0,
                motor_position_rad_left=0.5,
                motor_torque_nm_right=1.6,
                motor_speed_rad_per_sec_right=2.1,
                motor_position_rad_right=0.6,
            )
            writer.append_data(data)

        with tempfile.TemporaryDirectory() as tmpdir:
            filepath = writer.save_data(output_dir=Path(tmpdir))
            df = pd.read_csv(filepath)
            assert len(df) == num_rows


class TestSessionColumns:
    """The columns that make a recorded session replayable."""

    @staticmethod
    def _record(**overrides) -> RecordData:
        """Build a RecordData with the session columns filled.

        :param overrides: Field values to override.
        :return: A populated record.
        :rtype: RecordData
        """
        signal = SensorSignal(timestamp=1.0, angle_rad=0.7, velocity_rad_per_sec=0.2)
        fields = {
            "timestamp": 1.0,
            "raw_signal_left": signal,
            "filtered_signal_left": signal,
            "raw_signal_right": signal,
            "filtered_signal_right": signal,
            "motor_torque_nm_left": 0.0,
            "motor_speed_rad_per_sec_left": 0.0,
            "motor_position_rad_left": 0.0,
            "motor_torque_nm_right": 0.0,
            "motor_speed_rad_per_sec_right": 0.0,
            "motor_position_rad_right": 0.0,
            "motor_command_left": -1.25,
            "motor_command_right": 1.25,
            "operation_switch": True,
            "baseline_offset_rad_left": 0.33,
            "baseline_offset_rad_right": -0.21,
        }
        fields.update(overrides)
        return RecordData(**fields)

    def test_the_session_columns_are_written(self) -> None:
        """Switch, offsets and commands must reach the row, not just the dataclass."""
        writer = CSVWriter()
        writer.append_data(self._record())
        row = writer.rows[0]

        assert row[RecordDataColumnNames.OPERATION_SWITCH.value] == 1.0
        assert row[RecordDataColumnNames.BASELINE_OFFSET_RAD_LEFT.value] == 0.33
        assert row[RecordDataColumnNames.BASELINE_OFFSET_RAD_RIGHT.value] == -0.21
        assert row[RecordDataColumnNames.MOTOR_COMMAND_LEFT.value] == -1.25
        assert row[RecordDataColumnNames.MOTOR_COMMAND_RIGHT.value] == 1.25

    def test_the_switch_is_written_as_a_number(self) -> None:
        """A bool would round-trip through pandas as True/False, not 1/0.

        The playback side reads this column as a numeric trigger, so it has to
        be a number in the file.
        """
        writer = CSVWriter()
        writer.append_data(self._record(operation_switch=False))

        value = writer.rows[0][RecordDataColumnNames.OPERATION_SWITCH.value]
        assert value == 0.0
        assert isinstance(value, float)

    def test_the_raw_angle_stays_the_uncorrected_one(self) -> None:
        """The writer must record what it is given, offset included.

        Replay reproduces a session by feeding the raw angle back with the
        recorded trigger. If the raw column were ever the baseline-corrected
        angle, playback would subtract the offset a second time.
        """
        raw = SensorSignal(timestamp=1.0, angle_rad=0.7, velocity_rad_per_sec=0.2)
        corrected = SensorSignal(
            timestamp=1.0, angle_rad=0.7 - 0.33, velocity_rad_per_sec=0.2
        )

        writer = CSVWriter()
        writer.append_data(
            self._record(raw_signal_left=raw, filtered_signal_left=corrected)
        )
        row = writer.rows[0]

        assert row[RecordDataColumnNames.RAW_ANGLE_LEFT.value] == 0.7
        assert row[RecordDataColumnNames.RAW_ANGLE_LEFT.value] - row[
            RecordDataColumnNames.BASELINE_OFFSET_RAD_LEFT.value
        ] == pytest.approx(row[RecordDataColumnNames.FILTERED_ANGLE_LEFT.value])

    def test_saved_file_contains_the_session_columns(self) -> None:
        """End to end: the columns survive the DataFrame round-trip."""
        writer = CSVWriter()
        writer.append_data(self._record())

        with tempfile.TemporaryDirectory() as directory:
            path = writer.save_data(output_dir=Path(directory))
            frame = pd.read_csv(path)

        for column in (
            RecordDataColumnNames.OPERATION_SWITCH,
            RecordDataColumnNames.BASELINE_OFFSET_RAD_LEFT,
            RecordDataColumnNames.BASELINE_OFFSET_RAD_RIGHT,
            RecordDataColumnNames.MOTOR_COMMAND_LEFT,
            RecordDataColumnNames.MOTOR_COMMAND_RIGHT,
        ):
            assert column.value in frame.columns
        assert frame[RecordDataColumnNames.OPERATION_SWITCH.value].iloc[0] == 1.0
