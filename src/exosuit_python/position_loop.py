"""The outer position loop, wired as ``Control_ML_Stairs_IMUbased_developer.slx``.

The block that produces the controller's output is named, in the model,
**MOTOR POSITION REFERENCE GENERATOR**, and its outport is ``Ref motion``. What
``WalkOnController.step`` returns is that signal: a motor *position* reference
in radians, saturated in the model to +/-600 deg. The MIT frame carries a
velocity. The model closes the gap with a PID per leg, and this module is that
stage.

The wiring, read off the model (``Subsystem1``, SID 7305)::

    MOTOR POSITION REFERENCE GENERATOR ──► MC_L ──► PID1 "Ref"
                                                      │
    CAN Receive ─► Unpack2 ─► CHINESE CORRECTION ─┬─► PID1 "Actual Motion"
                                                  └─► MOTOR POSITION L (logged)
                                                      │
                            SWITCH ──► PID1 "Enable"   ▼
                                       PID1 ──► CAN Pack ──► motor velocity

Three properties of that diagram are easy to lose and all three matter:

* the PID's feedback is the motor's own position, not its velocity;
* the derivative term is not the derivative of the error. ``Gain7`` is fed from
  the *second* outport of a masked second-order low-pass filter (wn=20, zt=1)
  whose input is the PID's own previous output, and it is subtracted. The
  filter's first outport is terminated. This is velocity-feedback damping, and
  a textbook PID in its place would differentiate the error instead;
* the enable port carries ``StatesWhenEnabling: reset``, so the integrator and
  that filter start from zero every time the switch goes on.

The control law itself is hip-controller's :class:`PIDController`, which
implements exactly this block and which, until it was wired up here, nothing in
the package imported.

Three things differ from ``motor_control.py``, all deliberate:

* the rig calls its PID with no ``dt``, so it runs on a fixed 0.01 s as the
  model's ode1 solver does. This loop does not hold its nominal period -- a
  bench run measured a 95th percentile of 15.18 ms against a 10 ms target -- so
  it is stepped with the measured time, clamped, as the tensioning chart in
  this package already is;
* the rig resets its PIDs only in a whole-exosuit ``reset()``, so a session
  that toggles the switch carries the previous one's damping state across.
  The model's enable port says ``StatesWhenEnabling: reset``, and this follows
  the model;
* hip-controller's ``compute_output`` returns zero on its first call, having no
  previous timestamp to take a step from, where the rig produces a command
  immediately. That costs one tick of assist at the start of a session.
"""

import math

from hip_controller.control.motor_reference_control.pid_controller import PIDController
from hip_controller.definitions import LowPassFilterConfig, PIDConfig

from exosuit_python.definitions import PositionLoopConfig


class MotorPositionLoop:
    """Turn a motor position reference into a velocity command, for one leg.

    One instance per leg, as the model has one PID per leg with identical
    gains: the filter, the integrator, the clock and the unwrapping all carry
    per-leg state.
    """

    def __init__(self, config: PositionLoopConfig | None = None) -> None:
        """Build the loop for one leg.

        :param config: Loop gains, filter settings and limits. Defaults to
            ``PositionLoopConfig()``.
        :return: None
        """
        self._config = config if config is not None else PositionLoopConfig()
        self._pid = self._build_pid()
        # The PID derives its own time step from successive timestamps, so it
        # runs off a clock this class advances rather than the wall clock --
        # which is how the step stays bounded when a tick runs late.
        self._clock = 0.0
        self._was_enabled = False
        self._wrap_correction = 0.0
        self._previous_raw_position: float | None = None

    @property
    def position_rad(self) -> float:
        """Where the shaft is, continuous across the position field's rollover."""
        if self._previous_raw_position is None:
            return 0.0
        return self._previous_raw_position - self._wrap_correction

    def _build_pid(self) -> PIDController:
        """Return a PID at this loop's gains, with no state behind it.

        :return: A freshly built controller.
        :rtype: PIDController
        """
        return PIDController(
            pid_config=PIDConfig(
                proportional_gain=self._config.proportional_gain,
                integral_gain=self._config.integral_gain,
                derivative_gain=self._config.damping_gain,
                # Left unset on purpose: the rig saturates downstream, after
                # the value has been fed back to the damping filter. See
                # PositionLoopConfig.
                output_limits=None,
            ),
            filter_config=LowPassFilterConfig(
                cut_off_frequency_rad_per_sec=self._config.output_lpf_cutoff_rad_per_sec,
                damping_ratio=self._config.output_lpf_damping_ratio,
            ),
        )

    def reset(self) -> None:
        """Clear every piece of state, ready for another session.

        The model's enable port is set to reset its states, so this runs on the
        switch's rising edge as well as between sessions.

        The PID is rebuilt rather than reset. ``PIDController.reset()`` clears
        the integrator and the stored previous output, but leaves the damping
        filter holding the last session's state and leaves ``_prev_timestamp``
        set -- so the first tick after a reset would hand the filter a step
        measured from the old clock to the new one, which is negative. Building
        a new one is what the model's enable port actually does, and it costs a
        single allocation once per session.

        :return: None
        """
        self._pid = self._build_pid()
        self._clock = 0.0
        self._wrap_correction = 0.0
        self._previous_raw_position = None

    def unwrap(self, measured_rad: float) -> float:
        """Track the measured position across the position field's rollover.

        This is the model's ``CHINESE CORRECTION`` block, which sits between
        ``Unpack2`` and the PID's feedback input. A step larger than its
        threshold is taken as a rollover rather than motion, accumulated, and
        subtracted from the reported position, so the value handed to the loop
        is continuous.

        The threshold is the model's: a *step* of more than 20 rad between
        ticks. At the 41.87 rad/s the command saturates to, one tick of real
        motion is under half a radian, so only a rollover can reach it.

        A NaN reading -- no feedback yet -- holds the last known position
        rather than poisoning it, so the loop sees a stale position instead of
        an error of NaN.

        :param measured_rad: Position as the motor reports it, in radians.
        :return: The same position, continuous across rollovers.
        :rtype: float
        """
        if math.isnan(measured_rad):
            return self.position_rad

        if self._previous_raw_position is not None:
            step = measured_rad - self._previous_raw_position
            if abs(step) > self._config.wrap_detection_threshold_rad:
                self._wrap_correction += step
        self._previous_raw_position = measured_rad
        return measured_rad - self._wrap_correction

    def step(
        self,
        reference_rad: float,
        measured_rad: float,
        enabled: bool,
        time_difference: float,
    ) -> float:
        """Advance the loop by one control tick.

        :param reference_rad: Motor position reference from the controller, in
            radians -- the model's ``Ref motion`` / ``MC_L``.
        :param measured_rad: Position the motor reports, in radians.
        :param enabled: The model's SWITCH, wired to the PID's enable port. A
            disabled loop commands zero, and the rising edge clears its state
            because the enable port is set to reset.
        :param time_difference: Seconds since the previous tick.
        :return: Velocity command in rad/s at the output shaft.
        :rtype: float
        """
        if enabled and not self._was_enabled:
            # StatesWhenEnabling: reset. The integrator and the damping filter
            # start from zero, and so does the unwrapping -- pre-tensioning
            # re-zeroes the encoder as it ends, so the shaft is at its own zero
            # by the time the switch goes on.
            self.reset()
        self._was_enabled = enabled

        # Unwrapping runs whether or not the assist does. It records where the
        # shaft is rather than forming part of the control law.
        position = self.unwrap(measured_rad)

        if not enabled:
            return 0.0

        # A stall would otherwise hand the loop's 20 rad/s filter a step of
        # hundreds of milliseconds and rail it, which is the failure the SOGI
        # dt clamp exists for upstream. A late tick should cost accuracy, not
        # stability. The model has no equivalent because it runs fixed-step.
        self._clock += min(time_difference, self._config.max_time_step)
        return float(
            self._pid.compute_output(
                timestamp=self._clock,
                motor_reference=reference_rad,
                motor_position=position,
            )
        )
