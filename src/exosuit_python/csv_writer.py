"""Record data of the hip controller into a csv file."""

import csv
import math
import time
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import TextIO

import pandas as pd
from hip_controller.definitions import SensorSignal
from loguru import logger

from exosuit_python.definitions import RECORDINGS_DIR
from exosuit_python.utils import create_timestamped_filepath

# How often a streamed recording is flushed to disk. Per row would put a
# millisecond-scale syscall on the control thread; once a second bounds what a
# crash can cost to the last second of data.
FLUSH_INTERVAL_S = 1.0


@dataclass
class RecordData:
    """Container for the measurements from the sensor and motor of both lower limbs.

    :timestamp: current timestamp.
    :left: Signal state of the left lower limb.
    :right: Signal state of the right lower limb.
    """

    timestamp: float

    raw_signal_left: SensorSignal
    filtered_signal_left: SensorSignal

    raw_signal_right: SensorSignal
    filtered_signal_right: SensorSignal

    motor_torque_nm_left: float
    motor_speed_rad_per_sec_left: float
    motor_position_rad_left: float

    motor_torque_nm_right: float
    motor_speed_rad_per_sec_right: float
    motor_position_rad_right: float

    # Commanded value, as opposed to the measured telemetry above. This is what
    # the controller asked for, before the motor did anything about it.
    #
    # These and the fields below default so that callers with nothing to report
    # -- bench tools, tests -- need not invent values. The exosuit's own
    # recording path fills every one of them explicitly.
    motor_command_left: float = math.nan
    motor_command_right: float = math.nan

    # Operation switch, and the angle offset each limb was running with.
    # Recorded together so a session can be replayed exactly: the switch drives
    # baseline removal, and the offset says what it produced. The raw_signal_*
    # angles above are deliberately PRE-baseline -- logging the corrected angle
    # as raw would make playback subtract the offset twice.
    operation_switch: bool = False
    baseline_offset_rad_left: float = 0.0
    baseline_offset_rad_right: float = 0.0

    # Recorded alongside the operation switch because the pair determines what
    # the exosuit was doing: both off is standby, tension alone is
    # pretensioning, operation is running. Logging the two inputs rather than
    # the derived state also avoids recording a status that lags them.
    tension_switch: bool = False

    # 0 when the motor reports no fault. NaN when nothing read it.
    motor_error_left: float = math.nan
    motor_error_right: float = math.nan


class RecordDataColumnNames(StrEnum):
    """Container for the measurements from the sensor of both lower limbs."""

    TIMESTAMP = "time (s)"

    RAW_ANGLE_LEFT = "raw_angle_left (rad)"
    RAW_VELOCITY_LEFT = "raw_velocity_left (rad/s)"
    FILTERED_ANGLE_LEFT = "filtered_angle_left (rad)"
    FILTERED_VELOCITY_LEFT = "filtered_velocity_left (rad/s)"

    RAW_ANGLE_RIGHT = "raw_angle_right (rad)"
    RAW_VELOCITY_RIGHT = "raw_velocity_right (rad/s)"
    FILTERED_ANGLE_RIGHT = "filtered_angle_right (rad)"
    FILTERED_VELOCITY_RIGHT = "filtered_velocity_right (rad/s)"

    MOTOR_TORQUE_NM_LEFT = "motor_torque_left (Nm)"
    MOTOR_SPEED_RAD_PER_SEC_LEFT = "motor_speed_left (rad/s)"
    MOTOR_POSITION_RAD_LEFT = "motor_position_left (rad)"

    MOTOR_TORQUE_NM_RIGHT = "motor_torque_right (Nm)"
    MOTOR_SPEED_RAD_PER_SEC_RIGHT = "motor_speed_right (rad/s)"
    MOTOR_POSITION_RAD_RIGHT = "motor_position_right (rad)"

    MOTOR_COMMAND_LEFT = "motor_command_left (rad/s)"
    MOTOR_COMMAND_RIGHT = "motor_command_right (rad/s)"

    OPERATION_SWITCH = "operation_switch"
    BASELINE_OFFSET_RAD_LEFT = "baseline_offset_left (rad)"
    BASELINE_OFFSET_RAD_RIGHT = "baseline_offset_right (rad)"
    TENSION_SWITCH = "tension_switch"

    # The motor's own fault code, 0 when healthy. One column, and it
    # distinguishes a motor that refused a command from one that never got it.
    MOTOR_ERROR_LEFT = "motor_error_left"
    MOTOR_ERROR_RIGHT = "motor_error_right"


class CSVWriter:
    """Record data into a CSV file, buffered or streamed.

    Two modes. By default rows are held in memory and written by
    :meth:`save_data`, which suits offline tools that build a recording and
    write it once.

    :meth:`start_streaming` switches to writing each row straight to an open
    file. That is what the exosuit uses: a run records continuously, including
    standby, so buffering would grow without bound -- roughly 13 MB per minute
    at 100 Hz -- and a crash would take the whole recording with it. Streaming
    keeps memory flat and loses at most the last unflushed second.
    """

    def __init__(self) -> None:
        """Initialize the CSV writer."""
        self.rows: list[dict[str, float]] = []
        self._file: TextIO | None = None
        self._writer: csv.DictWriter | None = None
        self._last_flush_s: float = 0.0

    @property
    def is_streaming(self) -> bool:
        """Whether rows are being written straight to a file.

        :return: True between start_streaming() and stop_streaming().
        :rtype: bool
        """
        return self._file is not None

    def start_streaming(self, output_dir: Path = RECORDINGS_DIR) -> Path:
        """Open a timestamped file and write rows to it as they arrive.

        :param Path output_dir: Directory to create the recording in.
        :return: Path of the file being written.
        :rtype: Path
        """
        filepath = create_timestamped_filepath(
            output_dir=output_dir, prefix="recording_file", suffix="csv"
        )
        filepath.parent.mkdir(parents=True, exist_ok=True)

        self._file = filepath.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(
            self._file, fieldnames=[col.value for col in RecordDataColumnNames]
        )
        self._writer.writeheader()
        self._last_flush_s = time.monotonic()

        logger.info(f"Recording to '{filepath}'.")
        return filepath

    def stop_streaming(self) -> None:
        """Flush and close the streamed file, if one is open.

        :return: None
        """
        if self._file is None:
            return

        self._file.flush()
        self._file.close()
        self._file = None
        self._writer = None

    def reset(self) -> None:
        """Reset the CSV Writer."""
        self.rows = []

    def append_data(self, data: RecordData) -> None:
        """Append one line of data.

        Streamed straight to the file when streaming, otherwise buffered for
        :meth:`save_data`. The file is flushed at most once a second rather
        than per row: a buffered write costs microseconds, a flush costs
        milliseconds, and this runs on the control thread.
        """
        row = self._as_row(data)

        if self._writer is not None and self._file is not None:
            self._writer.writerow(row)
            now = time.monotonic()
            if now - self._last_flush_s >= FLUSH_INTERVAL_S:
                self._file.flush()
                self._last_flush_s = now
            return

        self.rows.append(row)

    @staticmethod
    def _as_row(data: RecordData) -> dict[str, float]:
        """Flatten one record into the column mapping written to the CSV.

        :param RecordData data: Record to flatten.
        :return: Column name to value.
        :rtype: dict[str, float]
        """
        return {
            RecordDataColumnNames.TIMESTAMP.value: data.timestamp,
            RecordDataColumnNames.RAW_ANGLE_LEFT.value: data.raw_signal_left.angle_rad,
            RecordDataColumnNames.RAW_VELOCITY_LEFT.value: data.raw_signal_left.velocity_rad_per_sec,
            RecordDataColumnNames.FILTERED_ANGLE_LEFT.value: data.filtered_signal_left.angle_rad,
            RecordDataColumnNames.FILTERED_VELOCITY_LEFT.value: data.filtered_signal_left.velocity_rad_per_sec,
            RecordDataColumnNames.RAW_ANGLE_RIGHT.value: data.raw_signal_right.angle_rad,
            RecordDataColumnNames.RAW_VELOCITY_RIGHT.value: data.raw_signal_right.velocity_rad_per_sec,
            RecordDataColumnNames.FILTERED_ANGLE_RIGHT.value: data.filtered_signal_right.angle_rad,
            RecordDataColumnNames.FILTERED_VELOCITY_RIGHT.value: data.filtered_signal_right.velocity_rad_per_sec,
            RecordDataColumnNames.MOTOR_TORQUE_NM_LEFT.value: data.motor_torque_nm_left,
            RecordDataColumnNames.MOTOR_TORQUE_NM_RIGHT.value: data.motor_torque_nm_right,
            RecordDataColumnNames.MOTOR_SPEED_RAD_PER_SEC_LEFT.value: data.motor_speed_rad_per_sec_left,
            RecordDataColumnNames.MOTOR_SPEED_RAD_PER_SEC_RIGHT.value: data.motor_speed_rad_per_sec_right,
            RecordDataColumnNames.MOTOR_POSITION_RAD_LEFT.value: data.motor_position_rad_left,
            RecordDataColumnNames.MOTOR_POSITION_RAD_RIGHT.value: data.motor_position_rad_right,
            RecordDataColumnNames.MOTOR_COMMAND_LEFT.value: data.motor_command_left,
            RecordDataColumnNames.MOTOR_COMMAND_RIGHT.value: data.motor_command_right,
            RecordDataColumnNames.OPERATION_SWITCH.value: float(data.operation_switch),
            RecordDataColumnNames.BASELINE_OFFSET_RAD_LEFT.value: data.baseline_offset_rad_left,
            RecordDataColumnNames.BASELINE_OFFSET_RAD_RIGHT.value: data.baseline_offset_rad_right,
            RecordDataColumnNames.TENSION_SWITCH.value: float(data.tension_switch),
            RecordDataColumnNames.MOTOR_ERROR_LEFT.value: data.motor_error_left,
            RecordDataColumnNames.MOTOR_ERROR_RIGHT.value: data.motor_error_right,
        }

    def save_data(self, output_dir: Path = RECORDINGS_DIR) -> Path:
        """Write data into the given path when the run is stopped."""
        # Initialize an empty dataframe with proper columns
        dataframe = pd.DataFrame(
            {col.value: pd.Series(dtype="float64") for col in RecordDataColumnNames}
        )

        # Concat list of data to dataframe
        dataframe = pd.concat([dataframe, pd.DataFrame(self.rows)], ignore_index=True)

        # Create timestamped file and directory
        filepath = create_timestamped_filepath(
            output_dir=output_dir, prefix="recording_file", suffix="csv"
        )
        filepath.parent.mkdir(parents=True, exist_ok=True)

        logger.info(f"Saving exosuit recording to '{filepath}'.")

        # Save the dataFrame to a CSV file
        dataframe.to_csv(filepath, index=False)

        return filepath
