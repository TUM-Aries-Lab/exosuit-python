"""Pre-tensioning, ported from ``motor_control.py`` (Subsystem3).

The bench-validated rig filters the motor's torque feedback, rectifies it, and
feeds it to a four-state Stateflow chart that pulls the tendon until the
torque crosses a threshold. This module reproduces that chart one-for-one, so
the suit behaves the way the rig does.

The chart is deliberately *not* collapsed into a simpler conditional. Two of
its properties matter and are easy to lose:

* releasing the switch leaves TENSIONING for DISABLE directly, without waiting
  for the torque threshold, so the wearer can always abort the pull;
* reaching the threshold passes through STOP, which holds the motor still for
  ``stop_hold_time`` before disabling, rather than cutting the command dead.
"""

from loguru import logger
from motor_python.definitions import LowPassFilterConfig
from motor_python.second_order_low_pass_filter import SecondOrderLowPassFilter

from exosuit_python.definitions import TensionConfig, TensionState


class LegTensioner:
    """Torque low-pass filter, rectifier and chart for a single leg.

    Transitions, verbatim from the chart:

    ===============  ==========  ==================================
    From             To          When
    ===============  ==========  ==================================
    (default)        RESET       -
    RESET            TENSIONING  ``on_off == 1``
    TENSIONING       DISABLE     ``on_off == 0``
    TENSIONING       STOP        ``|LPF(torque)| >= threshold``
    STOP             DISABLE     after ``stop_hold_time`` seconds
    ===============  ==========  ==================================

    On-entry actions, including the two deliberate omissions:

    * ``RESET``: velocity 0, enable 0.
    * ``TENSIONING``: velocity = the leg's tensioning velocity, enable 1.
    * ``STOP``: velocity 0. Enable is *not* touched, so it stays 1.
    * ``DISABLE``: enable -1. Velocity is *not* touched, so it holds its
      last value.
    """

    def __init__(
        self,
        tensioning_velocity_rad_per_sec: float,
        config: TensionConfig | None = None,
    ) -> None:
        """Build the tensioner for one leg.

        :param tensioning_velocity_rad_per_sec: Signed velocity command for
            this leg. The legs are mirrored, so left and right differ in sign.
        :param config: Tensioning parameters. Defaults to ``TensionConfig()``.
        :return: None
        """
        self._config = config if config is not None else TensionConfig()
        self._tensioning_velocity = tensioning_velocity_rad_per_sec
        self._filter = SecondOrderLowPassFilter(
            LowPassFilterConfig(
                cut_off_frequency_rad_per_sec=self._config.torque_lpf_cutoff_rad_per_sec,
                damping_ratio=self._config.torque_lpf_damping_ratio,
            )
        )
        self.state = TensionState.RESET
        self.velocity_rad_per_sec = 0.0
        self.enable = 0
        #: Whether ``enable`` changed on the last ``step``. Callers use this to
        #: act on the transition only. Disabling a motor is expensive -- the
        #: CAN ``stop()`` blocks for 60 ms of settling -- and the rig fires its
        #: enable/disable frames on edges for the same reason.
        self.enable_changed = False
        self._stop_timer = 0.0

    @property
    def finished(self) -> bool:
        """Whether this leg has reached its terminal state."""
        return self.state is TensionState.DISABLE

    def reset(self) -> None:
        """Return to RESET and clear the filter, ready for another session.

        :return: None
        """
        self._filter.reset()
        self.state = TensionState.RESET
        self.velocity_rad_per_sec = 0.0
        self.enable = 0
        self.enable_changed = False
        self._stop_timer = 0.0

    def step(
        self, torque_nm: float, on_off: int, time_difference: float
    ) -> tuple[float, int]:
        """Advance the chart by one control tick.

        :param torque_nm: Raw torque feedback for this leg, in N*m.
        :param on_off: 1 while the tension switch is held, 0 otherwise.
        :param time_difference: Seconds since the previous tick.
        :return: ``(velocity_rad_per_sec, enable)``, where enable is 1 while
            running, 0 while idle and -1 once disabled.
        """
        filtered_torque, _ = self._filter.step(torque_nm, time_difference)
        motor_torque = abs(filtered_torque)
        previous_enable = self.enable

        if self.state is TensionState.RESET:
            self.velocity_rad_per_sec = 0.0
            self.enable = 0
            if on_off == 1:
                self.state = TensionState.TENSIONING
                self.velocity_rad_per_sec = self._tensioning_velocity
                self.enable = 1

        elif self.state is TensionState.TENSIONING:
            # Transition order matters and matches the chart: the switch is
            # checked before the threshold, so releasing it always wins.
            if on_off == 0:
                self.state = TensionState.DISABLE
                self.enable = -1
            elif motor_torque >= self._config.torque_threshold_nm:
                logger.info(
                    f"Tension threshold reached: {motor_torque:.3f} N*m "
                    f">= {self._config.torque_threshold_nm} N*m."
                )
                self.state = TensionState.STOP
                self.velocity_rad_per_sec = 0.0
                self._stop_timer = 0.0

        elif self.state is TensionState.STOP:
            self.velocity_rad_per_sec = 0.0
            self._stop_timer += time_difference
            if self._stop_timer >= self._config.stop_hold_time:
                self.state = TensionState.DISABLE
                self.enable = -1

        self.enable_changed = self.enable != previous_enable
        return self.velocity_rad_per_sec, self.enable
