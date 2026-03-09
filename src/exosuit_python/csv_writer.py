"""Record data of the hip controller into a csv file."""

from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

import pandas as pd
from loguru import logger

from exosuit_python.definitions import RECORDINGS_DIR
from exosuit_python.utils import create_timestamped_filepath


# TODO replace this dataclass with import from definitions of hip controller
@dataclass
class SensorSignal:
    """Angle and velocity signal values."""

    angle_rad: float = 0.0
    velocity_rad_per_sec: float = 0.0


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

    motor_torque_nm_per_kg_left: float
    motor_speed_rad_per_sec_left: float
    motor_position_rad_left: float

    motor_torque_nm_per_kg_right: float
    motor_speed_rad_per_sec_right: float
    motor_position_rad_right: float


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

    MOTOR_TORQUE_NM_PER_KG_LEFT = "motor_torque_left (Nm/kg)"
    MOTOR_SPEED_RAD_PER_SEC_LEFT = "motor_speed_left (rad/s)"
    MOTOR_POSITION_RAD_LEFT = "motor_position_left (rad)"

    MOTOR_TORQUE_NM_PER_KG_RIGHT = "motor_torque_right (Nm/kg)"
    MOTOR_SPEED_RAD_PER_SEC_RIGHT = "motor_speed_right (rad/s)"
    MOTOR_POSITION_RAD_RIGHT = "motor_position_right (rad)"


class CSVWriter:
    """Record data into a CSV file."""

    def __init__(self) -> None:
        """Initialize the CSV writer."""
        self.rows: list[dict[str, float]] = []

    def reset(self) -> None:
        """Reset the CSV Writer."""
        self.rows = []

    def append_data(self, data: RecordData) -> None:
        """Append one line of data to the dataframe."""
        self.rows.append(
            {
                RecordDataColumnNames.TIMESTAMP.value: data.timestamp,
                RecordDataColumnNames.RAW_ANGLE_LEFT.value: data.raw_signal_left.angle_rad,
                RecordDataColumnNames.RAW_VELOCITY_LEFT.value: data.raw_signal_left.velocity_rad_per_sec,
                RecordDataColumnNames.FILTERED_ANGLE_LEFT.value: data.filtered_signal_left.angle_rad,
                RecordDataColumnNames.FILTERED_VELOCITY_LEFT.value: data.filtered_signal_left.velocity_rad_per_sec,
                RecordDataColumnNames.RAW_ANGLE_RIGHT.value: data.raw_signal_right.angle_rad,
                RecordDataColumnNames.RAW_VELOCITY_RIGHT.value: data.raw_signal_right.velocity_rad_per_sec,
                RecordDataColumnNames.FILTERED_ANGLE_RIGHT.value: data.filtered_signal_right.angle_rad,
                RecordDataColumnNames.FILTERED_VELOCITY_RIGHT.value: data.filtered_signal_right.velocity_rad_per_sec,
                RecordDataColumnNames.MOTOR_TORQUE_NM_PER_KG_LEFT.value: data.motor_torque_nm_per_kg_left,
                RecordDataColumnNames.MOTOR_TORQUE_NM_PER_KG_RIGHT.value: data.motor_torque_nm_per_kg_right,
                RecordDataColumnNames.MOTOR_SPEED_RAD_PER_SEC_LEFT.value: data.motor_speed_rad_per_sec_left,
                RecordDataColumnNames.MOTOR_SPEED_RAD_PER_SEC_RIGHT.value: data.motor_speed_rad_per_sec_right,
                RecordDataColumnNames.MOTOR_POSITION_RAD_LEFT.value: data.motor_position_rad_left,
                RecordDataColumnNames.MOTOR_POSITION_RAD_RIGHT.value: data.motor_position_rad_right,
            }
        )

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
