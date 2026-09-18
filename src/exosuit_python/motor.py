"""Mock motor for CI testing."""

from typing import Any

from loguru import logger


class MockMotor:
    """Mock motor class.

    Mirrors the parts of ``CubeMarsAK806v2CAN`` that the exosuit calls. Note
    that ``get_current`` returns a *torque* in N*m: the real CAN class decodes
    the MIT feedback field against the motor profile's ``t_min``/``t_max``, so
    the quantity is a torque even though the method says current.
    """

    #: N*m added per tick while a tensioning velocity is commanded. Sized so a
    #: mock pull crosses TensionConfig.torque_threshold_nm in a fraction of a
    #: second at 100 Hz -- quick enough for tests, not instantaneous.
    TORQUE_RAMP_NM_PER_TICK = 0.05

    def __init__(self) -> None:
        """Initialize the mock motor with no load on the tendon."""
        self._torque_nm = 0.0
        #: The last MIT command received, as a dict, or None. Tests assert on
        #: this to pin the units of the command path.
        self.last_mit_command: dict[str, float] | None = None
        #: Counters the safety tests read. A released motor and a motor that
        #: was merely commanded to zero look identical from last_mit_command,
        #: and "did it pull again?" cannot be answered from a single value at
        #: all, so both are counted.
        self.stop_calls = 0
        self.pull_commands = 0
        self.get_current_calls = 0
        #: The feedback cache the CAN transport keeps, which the exosuit reads
        #: in preference to sending a request. Present here so tests can
        #: exercise both the cached and the fallback path; None means "no
        #: cache", which is the plain mock's normal state.
        self._last_feedback: Any | None = None
        self._last_feedback_monotonic: float = 0.0

    def set_velocity(self, velocity_erpm: int) -> None:
        """Set mock motor velocity."""
        logger.debug(f"Velocity set: {velocity_erpm}")

    def set_mit_mode(
        self,
        pos_rad: float = 0.0,
        vel_rad_s: float = 0.0,
        kp: float = 0.0,
        kd: float = 0.0,
        torque_ff_nm: float = 0.0,
    ) -> None:
        """Accept an MIT command and build tension while the tendon is pulled.

        A real tendon loads up as the motor winds it in, which is what the
        tensioning chart thresholds on. Ramping here keeps that path
        exercisable on mock devices instead of stalling forever at zero.
        """
        logger.debug(f"MIT command: vel={vel_rad_s} rad/s, kd={kd}")
        if vel_rad_s != 0.0:
            self.pull_commands += 1
        self.last_mit_command = {
            "pos_rad": pos_rad,
            "vel_rad_s": vel_rad_s,
            "kp": kp,
            "kd": kd,
            "torque_ff_nm": torque_ff_nm,
        }
        if vel_rad_s != 0.0:
            self._torque_nm += self.TORQUE_RAMP_NM_PER_TICK

    def get_current(self) -> float | None:
        """Return the MIT torque feedback in N*m (see the class docstring).

        Counted, because on the real CAN motor the status request frame is
        byte-identical to the MIT enable frame -- so polling a stopped motor
        re-energises it, and "was it polled at all?" is a safety question.
        """
        self.get_current_calls += 1
        return self._torque_nm

    def stop(self) -> None:
        """Release the mock motor.

        The tendon keeps its load. A real one does not go slack the instant
        the windings are released, and zeroing here would hide a controller
        that pulls the same tendon twice.
        """
        self.stop_calls += 1

    def close(self) -> None:
        """Close mock motor connection."""

    def check_communication(self) -> bool:
        """Check if motor can communicate."""
        return True
