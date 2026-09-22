"""Exosuit configuration."""

import math
import threading
import time
import warnings
from dataclasses import dataclass, field

from loguru import logger

try:
    from Jetson import GPIO
except Exception:
    logger.warning("Jetson GPIO import failed. Are you running on the Jetson?")
    GPIO = None
from hip_controller.control.app import WalkOnController
from hip_controller.definitions import BasicConfig, SensorSignal
from imu_python.factory import IMUFactory
from imu_python.sensor_manager import IMUManager
from motor_python.cube_mars_motor_can import CubeMarsAK806v2CAN

from exosuit_python.can_bringup import ensure_can_interface
from exosuit_python.csv_writer import CSVWriter, RecordData
from exosuit_python.definitions import (
    BOTH,
    EXOSUIT_STANDBY_INTERVAL,
    GPIO_SWITCH_BOUNCETIME,
    LOCOMOTION_CLASS_IDS,
    MODE_SWITCH_1,
    MODE_SWITCH_2,
    MODE_SWITCH_LOGIC,
    MOTOR_CAN_ID_LEFT,
    MOTOR_CAN_ID_RIGHT,
    OPERATION_SWITCH,
    SWITCH_EVENT_HANDLER_INTERVAL,
    TENSION_SWITCH,
    THREAD_JOIN_TIMEOUT,
    ExosuitStates,
    IMUConfig,
    IMUMounting,
    InclinationModes,
    MotorCommandConfig,
    MotorSaturation,
    SwitchStates,
    TensionConfig,
)
from exosuit_python.gpio import MockGPIO
from exosuit_python.motor import MockMotor
from exosuit_python.motor_watchdog import MotorDropoutWatchdog, WatchdogVerdict
from exosuit_python.position_loop import MotorPositionLoop
from exosuit_python.tensioning import LegTensioner


def _saturate(value: float, low: float, high: float) -> float:
    """Clamp a command into its permitted range.

    :param value: The requested value.
    :param low: Lower limit, inclusive.
    :param high: Upper limit, inclusive.
    :return: The value, clamped.
    """
    return max(low, min(high, value))


@dataclass(frozen=True)
class MotorTelemetry:
    """What a motor reports, converted into this package's units.

    :position_rad: Shaft position in radians.
    :velocity_rad_per_sec: Shaft velocity in radians per second.
    :torque_nm: Torque in N*m.
    :error_code: The motor's fault code, 0 when healthy.
    """

    position_rad: float
    velocity_rad_per_sec: float
    torque_nm: float
    error_code: float


@dataclass
class ExosuitConfig:
    """Exosuit configuration.

    Attributes:
        frequency: Exosuit frequency in Hz.
        mock_devices: flag to use mock devices.
        test_gpio: flag to use Jetson GPIO (for switch testing on the Jetson) and use mock_devices.
        imu_cfg: IMU config that defines the IMU to use for each leg.
        record: write a CSV recording of every session to RECORDINGS_DIR. On by
            default -- an unrecorded trial cannot be analysed afterwards, and
            the cost is one buffered row per control iteration.

    """

    frequency: float
    mock_devices: bool = False
    test_gpio: bool = False
    imu_cfg: IMUConfig = field(default_factory=IMUConfig)
    record: bool = True


class Exosuit:
    """Tendon-based soft exoskeleton."""

    def __init__(self, config: ExosuitConfig) -> None:
        """Initialize the exosuit.

        :param config: Exosuit configuration
        """
        self.config: ExosuitConfig = config
        self._status: ExosuitStates = ExosuitStates.INITIALIZING

        # GPIO for switches
        if GPIO is None or (not self.config.test_gpio and self.config.mock_devices):
            self.gpio = MockGPIO()
        else:
            self.gpio = GPIO

        if (
            hasattr(self.gpio, SwitchStates.ON)
            and hasattr(self.gpio, SwitchStates.OFF)
            and hasattr(self.gpio, BOTH)
        ):
            self.on_signal = getattr(self.gpio, SwitchStates.ON)
            self.off_signal = getattr(self.gpio, SwitchStates.OFF)
            self.both_signal = getattr(self.gpio, BOTH)
        else:
            logger.error(
                f"GPIO signal attribute '{SwitchStates.ON}', '{SwitchStates.OFF}', or '{BOTH}' not found."
            )
            return

        self._operation_switch: bool = False
        self._tension_switch: bool = False

        # Set by the GPIO callbacks so the switch handler reacts to an edge
        # instead of discovering it on its next poll. The handler still wakes on
        # a timeout as well, because the mode switches are polled rather than
        # edge-detected -- see _switch_event_handler.
        self._switch_event: threading.Event = threading.Event()

        # main loop thread and switch handler thread
        self.thread: threading.Thread = threading.Thread(target=self._loop, daemon=True)
        self.switch_thread: threading.Thread = threading.Thread(
            target=self._switch_event_handler, daemon=True
        )
        self.inclination_mode: InclinationModes = InclinationModes.LEVEL_GROUND
        self._prev_inclination_mode: InclinationModes = InclinationModes.LEVEL_GROUND

        self.imu_left: IMUManager
        self.imu_right: IMUManager

        # One controller configuration shared by both limbs. `frequency` must
        # be the real loop rate: hip-controller derives its sample-rate
        # dependent filters (notches, baseline window) from it, so a mismatch
        # silently mistunes them. `filtered=False` because the signal handed to
        # step() is the raw IMU angle -- the pre-processing pipeline runs inside
        # the controller. `right_limb_reverse=False` overrides hip-controller's
        # default, which mirrors the right limb with -1 for rigs whose two
        # motors are mounted opposite each other. This suit's are not, and its
        # sensors do not mirror either: bench recordings on 2026-09-18 showed
        # a positive command winding the cable in on both motors, and flexion
        # raising the angle and the velocity on both legs alike. With nothing
        # inverted there is nothing for the flag to cancel, and leaving it set
        # would have inverted the right leg's assist on its own.
        #
        # Overridden here rather than changed upstream because it describes
        # this exo's build, not the controller.
        self.controller_config = BasicConfig(
            frequency=int(config.frequency),
            filtered=False,
            right_limb_reverse=False,
        )
        self.controller_left = WalkOnController(
            left_limb=True, config=self.controller_config
        )
        self.controller_right = WalkOnController(
            left_limb=False, config=self.controller_config
        )

        # One writer, reset per session, so each run of the operation switch
        # produces its own timestamped CSV rather than one file per process.
        self.csv_writer = CSVWriter()

        # Mock sensors carry their own names and share a bus, so the
        # hardware config cannot match them; see IMUConfig.for_mock_devices.
        # Chosen here rather than by the caller because with mock devices
        # there is only one config that can work.
        self._imu_cfg = (
            IMUConfig.for_mock_devices()
            if (self.config.mock_devices or self.config.test_gpio)
            else self.config.imu_cfg
        )

        self.motor_left: CubeMarsAK806v2CAN | MockMotor
        self.motor_right: CubeMarsAK806v2CAN | MockMotor

        if self.config.mock_devices or self.config.test_gpio:
            self.motor_left = MockMotor()
            self.motor_right = MockMotor()
        else:
            # Before the motors exist, because constructing them opens the bus:
            # a CAN interface that is down produces "Motor not responding",
            # which reads as a wiring fault and is one command away from fixed.
            # A failure here is logged, not raised -- _initialize_motors
            # reports the real state of the link a moment later, and a suit
            # that cannot reach its motors should say so once, in its own
            # words.
            ensure_can_interface()

            # AK80-6 over CAN. The MIT force-control protocol is what makes
            # pre-tensioning possible at all: its feedback frame carries the
            # torque the tensioning chart thresholds on. The UART servo path
            # reports phase current instead, and cannot address two motors on
            # one line -- its frame has no node ID field.
            self.motor_left = CubeMarsAK806v2CAN(
                motor_can_id=MOTOR_CAN_ID_LEFT,
                mit_velocity_kd=MotorCommandConfig.velocity_kd,
            )
            self.motor_right = CubeMarsAK806v2CAN(
                motor_can_id=MOTOR_CAN_ID_RIGHT,
                mit_velocity_kd=MotorCommandConfig.velocity_kd,
            )

        self._build_control_state()

        # initialization calls
        if not self._initialize_imus():
            logger.error("IMU initialization failed. Exosuit not started.")
            return
        if not self._initialize_motors():
            logger.error("Motor initialization failed. Exosuit not started.")
            return

        if self._initialize_gpio():
            self._start()
        else:
            logger.error("GPIO initialization failed. Exosuit not started.")

    def _initialize_gpio(self) -> bool:
        """Initialize and set up Jetson GPIO switches.

        :return: True if successful, False otherwise
        """
        try:
            # TEGRA_SOC, not BOARD: Blinka sets TEGRA_SOC when an IMU
            # driver is imported, and Jetson.GPIO permits one mode per
            # process. Agreeing with it makes its call a no-op; disagreeing
            # makes whichever runs second raise. See the pin definitions.
            self.gpio.setmode(self.gpio.TEGRA_SOC)
            self.gpio.setup(OPERATION_SWITCH, self.gpio.IN)
            self.gpio.setup(TENSION_SWITCH, self.gpio.IN)
            self.gpio.setup(MODE_SWITCH_1, self.gpio.IN)
            self.gpio.setup(MODE_SWITCH_2, self.gpio.IN)

            self.gpio.add_event_detect(
                OPERATION_SWITCH,
                self.both_signal,
                callback=self._operation_callback,
                bouncetime=GPIO_SWITCH_BOUNCETIME,
            )

            self.gpio.add_event_detect(
                TENSION_SWITCH,
                self.both_signal,
                callback=self._tension_callback,
                bouncetime=GPIO_SWITCH_BOUNCETIME,
            )

            return True
        except Exception as err:
            logger.error(f"GPIO init failure: {err}")
            self.gpio.cleanup()
            return False

    def _switch_event_handler(
        self,
    ) -> None:  # TODO: implement proper state machine if needed
        """Monitor switch states and handle exosuit status changes.

        :return: None
        """
        while self._status != ExosuitStates.STOPPED:
            self._read_switch_states()

            if self._status == ExosuitStates.STANDBY:
                if self._operation_switch and not self._tension_switch:
                    logger.info("State change: standby -> running")
                    self._status = ExosuitStates.RUNNING
                elif self._tension_switch and not self._operation_switch:
                    logger.info("State change: standby -> pretensioning")
                    self._status = ExosuitStates.PRETENSIONING
            elif self._status == ExosuitStates.RUNNING:
                if not self._operation_switch:
                    logger.info("State change: running -> standby")
                    self._status = ExosuitStates.STANDBY
            elif self._status == ExosuitStates.PRETENSIONING:
                pass  # state change handled in _loop()

            switch_1 = (
                SwitchStates.ON
                if self.gpio.input(MODE_SWITCH_1) == self.on_signal
                else SwitchStates.OFF
            )
            switch_2 = (
                SwitchStates.ON
                if self.gpio.input(MODE_SWITCH_2) == self.on_signal
                else SwitchStates.OFF
            )
            mode = self._get_mode(switch_1=switch_1, switch_2=switch_2)
            if mode is not None:
                self.inclination_mode = mode

            # Wake on the next switch edge, or on the timeout -- whichever comes
            # first. The timeout is what keeps the mode switches working: they
            # have no add_event_detect registered, so MODE_SWITCH_1/2 are read
            # by polling above and still need a periodic pass.
            #
            # An edge that arrives during the handling above is not missed: the
            # callback has already set the event, so wait() returns at once and
            # the next pass sees the new flag.
            self._switch_event.wait(timeout=SWITCH_EVENT_HANDLER_INTERVAL)
            self._switch_event.clear()

    def _operation_callback(self, channel: int) -> None:
        """Wake the switch handler on an operation-switch edge.

        Deliberately does not sample the pin -- see ``_read_switch_states``.
        """
        self._switch_event.set()

    def _tension_callback(self, channel: int) -> None:
        """Wake the switch handler on a tension-switch edge.

        Deliberately does not sample the pin -- see ``_read_switch_states``.
        """
        self._switch_event.set()

    def _read_switch_states(self) -> None:
        """Sample the two edge-detected switches and store their levels.

        This runs in the handler rather than in the GPIO callbacks, and the
        distinction is not cosmetic. A callback fires on the *first* edge of a
        bouncing contact, and GPIO_SWITCH_BOUNCETIME then drops every further
        edge on that pin for its duration -- including the one where the
        contact finally settles. Sampling inside the callback therefore reads
        a level that may still be mid-bounce and then never hears the
        correction, latching the wrong state until the next actuation.

        Reading here instead means the level is sampled after the wake-up,
        and re-sampled every SWITCH_EVENT_HANDLER_INTERVAL whether an edge
        arrived or not, so a dropped edge costs at most one pass.

        :return: None
        """
        operation_state = self.gpio.input(OPERATION_SWITCH)
        if operation_state == self.on_signal:
            self._operation_switch = True
        elif operation_state == self.off_signal:
            self._operation_switch = False
        else:
            logger.warning(f"Unrecognized operation switch state: {operation_state}")

        tension_state = self.gpio.input(TENSION_SWITCH)
        if tension_state == self.on_signal:
            self._tension_switch = True
        elif tension_state == self.off_signal:
            self._tension_switch = False
        else:
            logger.warning(f"Unrecognized tension switch state: {tension_state}")

    def _start(self) -> None:
        """Start the IMUs and Motors.

        :return: None
        """
        logger.info(f"Starting Exosuit at '{self.config.frequency}' Hz.")
        try:
            logger.debug("Starting IMUs")
            self.imu_left.start()
            self.imu_right.start()

            logger.debug("Starting Motors")
            logger.debug("Starting Controller")

            self._start_recording()

            # TODO: get mode
            self._status = ExosuitStates.STANDBY
            logger.info("Exosuit status: standby")
            # Start main control loop
            self.thread.start()
            self.switch_thread.start()

        except Exception as err:
            logger.info(f"Exosuit exception: '{err}'.")
            self._cleanup()

    def _cleanup(self) -> None:
        """Clean up the soft exoskeleton.

        :return: None
        """
        logger.info("Cleaning up exosuit.")
        self._status = ExosuitStates.STOPPED
        # Wake the handler so it sees STOPPED now rather than after its timeout.
        self._switch_event.set()
        with warnings.catch_warnings():
            # suppress warning from GPIO when no channels has been set up
            warnings.simplefilter("ignore", RuntimeWarning)
            self.gpio.cleanup()
        # Closed before the IMUs stop and before the loop thread is joined:
        # the standby lap reads the IMUs every tick, so stopping them first
        # leaves it logging read failures at 100 Hz until the join completes.
        self._save_recording()
        try:  # imu attributes can be unassigned in case of failure
            self.imu_left.stop()
            self.imu_right.stop()
        except AttributeError:
            pass
        self.motor_left.close()
        self.motor_right.close()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=THREAD_JOIN_TIMEOUT)
        if self.switch_thread is not None and self.switch_thread.is_alive():
            self.switch_thread.join(timeout=THREAD_JOIN_TIMEOUT)
        logger.success("Exosuit shutdown.")

    def _loop(self) -> None:
        """Run main control loop.

        :return: None
        """
        was_running = False
        while self._status != ExosuitStates.STOPPED:
            due = time.monotonic()
            while self._status == ExosuitStates.RUNNING:
                was_running = True
                try:
                    self._control()
                except TypeError as err:
                    logger.error(f"Failed getting data from the IMU: '{err}'.")
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

                due = self._sleep_until_due(due)

            # Session over. _control() cannot deliver the falling edge itself:
            # it stops being called the moment the status leaves RUNNING. Without
            # this the trigger stays high inside the controller, the next session
            # produces no rising edge, and no baseline is ever taken again --
            # baseline removal would work on the first run of the process and
            # silently never again. Idempotent: the controller acts on edges, so
            # repeating it while idle does nothing.
            self._set_baseline_removal_trigger(False)

            # Release the motors on the same falling edge. Nothing else does:
            # _control() stops being called the moment the status leaves
            # RUNNING, and set_mit_mode() installs a keep-alive thread that
            # re-transmits the last MIT payload until stop() or close(). So
            # without this both motors keep driving at the last assist command
            # after the operator has switched off, and an exception in
            # _control() freezes the assist at its last value instead of
            # dropping it. The old UART path had no keep-alive, which is why
            # this only became reachable with the move to CAN.
            #
            # Edge-triggered, like the rig's own enable/disable frames: stop()
            # blocks for the best part of a tenth of a second, and this block
            # runs on every lap of the outer loop, not only after a session.
            if was_running:
                was_running = False
                self._release_motors()
                self._reset_position_loops()

            # Pretensioning ticks at the control rate, like every other lap, so
            # the recording keeps one row spacing for the whole run. The chart
            # and its torque low-pass filter are stepped every tick for the
            # same reason the rig does: both are rate-dependent, and the STOP
            # hold is counted in ticks rather than slept through.
            due = time.monotonic()
            while self._status == ExosuitStates.PRETENSIONING:
                try:
                    self._pretension()
                    self._record_sensors_only()
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

                due = self._sleep_until_due(due)

            # Keep recording while idle, at the same rate, so one run produces
            # one continuous file at one spacing. Written once, at shutdown.
            due = time.monotonic()
            while self._status == ExosuitStates.STANDBY:
                try:
                    self._record_sensors_only()
                except Exception as err:
                    logger.error(f"Exosuit idle recording exception: '{err}'.")

                due = self._sleep_until_due(due)

            # Reached only in states with no loop of their own (INITIALIZING),
            # and keeps this from becoming a busy spin.
            time.sleep(EXOSUIT_STANDBY_INTERVAL)

    def _build_control_state(self) -> None:
        """Create the per-leg control blocks and the loop's own bookkeeping.

        Split out of ``__init__`` only for length: everything here is plain
        construction with no ordering constraints against the rest of it.

        :return: None
        """
        # One tensioning chart per leg. The legs are mirrored, so they pull in
        # opposite directions.
        self._tensioner_left = LegTensioner(
            TensionConfig.tensioning_velocity_left_rad_per_sec
        )
        self._tensioner_right = LegTensioner(
            TensionConfig.tensioning_velocity_right_rad_per_sec
        )

        # One position loop per leg. The controller's output is a motor
        # position reference in radians, and this is what turns it into the
        # velocity the MIT frame carries; see PositionLoopConfig.
        self._position_loop_left = MotorPositionLoop()
        self._position_loop_right = MotorPositionLoop()

        # One dropout watchdog per leg. A motor that trips its own protection
        # keeps answering commands without acting on them, and nothing else in
        # this loop would notice; see MotorWatchdogConfig.
        self._watchdog_left = MotorDropoutWatchdog()
        self._watchdog_right = MotorDropoutWatchdog()
        #: Which give-up already has a line in the log, keyed by leg.
        self._reported_give_up: dict[str, int] = {}

        # Wall clock of the previous assist tick, so the loop is stepped with
        # the time that actually passed. None between sessions.
        self._last_control_tick: float | None = None

        # Whether the last velocity command hit its limit, so saturation is
        # reported on the edge rather than once per tick.
        self._velocity_was_clamped = False

        # Wall clock of the previous pre-tensioning tick, so the chart is
        # stepped with the time that actually passed. None between sessions.
        self._last_pretension_tick: float | None = None

    def _set_baseline_removal_trigger(self, active: bool) -> None:
        """Drive baseline removal (hip angle offset) on both limbs.

        Called only from the control thread. The controller detects edges
        internally with a read-modify-write on state the control loop also
        touches, so driving it from the GPIO callback or the switch-handler
        thread would race; ``_operation_switch`` is a plain bool those threads
        assign, and reading it once per control iteration is safe.

        :param bool active: Current operation-switch state.
        :return: None
        """
        self.controller_left.set_baseline_removal_trigger(active=active)
        self.controller_right.set_baseline_removal_trigger(active=active)

    def _control(self) -> None:
        """Execute one iteration of control loop."""
        data_right = self.imu_right.get_data()
        data_left = self.imu_left.get_data()

        if self._prev_inclination_mode != self.inclination_mode:
            logger.info(
                f"Mode change:{self._prev_inclination_mode.name} -> {self.inclination_mode.name}"
            )
            # Both legs, and through set_locomotion_mode rather than reaching
            # into amplitude_modulation: it also swaps the per-mode SOGI-FLL
            # tuning and the motion-mapping table, which the old call left on
            # the level-ground settings whatever mode was selected.
            class_id = LOCOMOTION_CLASS_IDS[self.inclination_mode]
            self.controller_left.set_locomotion_mode(class_id)
            self.controller_right.set_locomotion_mode(class_id)
            self._prev_inclination_mode = self.inclination_mode

        if data_right is None or data_left is None:
            raise TypeError

        # The operation switch is the baseline-removal trigger: its rising edge
        # is the operator enabling the motors with the subject standing ready,
        # which is exactly when the angle offset should be taken. Driven before
        # stepping so the window is open for this sample.
        self._set_baseline_removal_trigger(self._operation_switch)

        signal_left, signal_right = self._build_signals(
            data_left=data_left, data_right=data_right
        )
        # What the controller returns is a motor position reference in
        # radians, not a velocity. The rig logs it as "Motor Ref Left [rad]"
        # and closes a loop on the motor's own position to get the velocity
        # the MIT frame carries; commanding it directly as a velocity
        # integrates it into an ever-tightening tendon. See PositionLoopConfig.
        reference_right = self.controller_right.step(curr_signal=signal_right)
        reference_left = self.controller_left.step(curr_signal=signal_left)

        time_difference = self._elapsed_since_last_control_tick()
        left_state = self._read_motor_state(self.motor_left)
        right_state = self._read_motor_state(self.motor_right)

        # The loop is stepped whether or not the assist is enabled. Disabled it
        # commands zero, but it keeps following the shaft, so a rollover that
        # happens while the assist is off is not lost; the switch's rising edge
        # is what clears its control state, as the model's enable port does.
        command_left = self._position_loop_left.step(
            reference_rad=reference_left,
            measured_rad=left_state.position_rad,
            enabled=self._operation_switch,
            time_difference=time_difference,
        )
        command_right = self._position_loop_right.step(
            reference_rad=reference_right,
            measured_rad=right_state.position_rad,
            enabled=self._operation_switch,
            time_difference=time_difference,
        )

        command_left = self._guard_motor(
            self.motor_left, self._watchdog_left, "left", command_left, left_state
        )
        command_right = self._guard_motor(
            self.motor_right, self._watchdog_right, "right", command_right, right_state
        )

        self._command_velocity(self.motor_left, command_left)
        self._command_velocity(self.motor_right, command_right)

        if self.config.record:
            self._record_sample(
                raw=(signal_left, signal_right),
                filtered=(
                    self.controller_left.last_filtered_signal or signal_left,
                    self.controller_right.last_filtered_signal or signal_right,
                ),
                commands=(command_left, command_right),
                references=(reference_left, reference_right),
                states=(left_state, right_state),
            )

    def _guard_motor(self, motor, watchdog, leg: str, command: float, state) -> float:
        """Return the command to actually send, after checking the motor is alive.

        A motor that has dropped out of MIT mode answers every frame and acts
        on none of them, so the loop sees a position that never changes, grows
        its error without bound and rails the command. Commanding a dead shaft
        is useless while it stays dead and dangerous the moment it wakes, since
        the standing command is whatever the error had grown to.

        :param motor: The motor for this leg.
        :param watchdog: That leg's watchdog.
        :param leg: Which leg, for the log.
        :param command: What the position loop asked for, in rad/s.
        :param state: This tick's telemetry for that motor.
        :return: The command to send, zero while the motor is not answering.
        :rtype: float
        """
        verdict = watchdog.step(
            command_rad_per_sec=command,
            speed_rad_per_sec=state.velocity_rad_per_sec,
            torque_nm=state.torque_nm,
        )
        if verdict is WatchdogVerdict.HEALTHY:
            return command

        if verdict is WatchdogVerdict.RECOVER:
            logger.warning(
                f"The {leg} motor is taking commands without moving "
                f"(attempt {watchdog.recovery_attempts}). Re-enabling MIT mode."
            )
            try:
                motor.enable_mit_mode()
            except Exception as err:
                logger.error(f"Could not re-enable the {leg} motor: '{err}'.")
            return 0.0

        # GIVE_UP is latched, so this would otherwise log every tick.
        if watchdog.recovery_attempts == self._reported_give_up.get(leg, -1):
            return 0.0
        self._reported_give_up[leg] = watchdog.recovery_attempts
        logger.error(
            f"The {leg} motor stopped responding and did not come back. "
            f"It is no longer being commanded; the other leg continues."
        )
        return 0.0

    def _command_velocity(
        self, motor, velocity_rad_per_sec: float, velocity_kd: float | None = None
    ) -> None:
        """Send a velocity-only MIT command, in rad/s at the output shaft.

        ``WalkOnController.step`` returns a motor velocity command in rad/s,
        and ``set_mit_mode`` takes rad/s, so the two meet directly. Going via
        ``set_velocity`` instead would mean converting into ERPM and straight
        back out, which is where two separate faults used to live:

        * ``convert_rad_per_sec_to_rpm`` produced *mechanical* RPM while the
          parameter it fed expects *electrical* RPM. The motor then divided by
          pole pairs times gear ratio (21 * 6), so every assist command was
          126 times too small -- and both quantities are plausible-looking
          ints, so nothing caught it.
        * ``set_velocity`` treats zero as a request to stop, and on this CAN
          class ``stop()`` disables MIT mode. The assist command crosses zero
          every stride, and anything under 0.105 rad/s truncated to zero, so
          MIT mode would have been torn down and rebuilt continuously.

        :param motor: The motor to command.
        :param velocity_rad_per_sec: Output-shaft velocity command in rad/s.
        :param velocity_kd: Damping gain for this command. Defaults to the
            assist gain; pre-tensioning passes its own, so the two regimes
            stay independently tunable.
        :return: None
        """
        # Saturate before the frame is packed, as the rig does inside
        # pack_mit_command(). Position, Kp and the feed-forward torque are
        # fixed at zero by this method and cannot leave their ranges, so only
        # the two values a caller can vary are clamped here.
        requested = velocity_rad_per_sec
        velocity_rad_per_sec = _saturate(
            requested, *MotorSaturation.velocity_rad_per_sec
        )
        # Edge-triggered: a railed controller holds its output, and warning on
        # every tick at 100 Hz would bury the moment it started.
        clamped = velocity_rad_per_sec != requested
        if clamped and not self._velocity_was_clamped:
            logger.warning(
                f"Velocity command {requested:.3f} rad/s saturated to "
                f"{velocity_rad_per_sec:.3f} rad/s."
            )
        elif self._velocity_was_clamped and not clamped:
            logger.info("Velocity command back within limits.")
        self._velocity_was_clamped = clamped

        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=velocity_rad_per_sec,
            kp=MotorCommandConfig.kp,
            kd=_saturate(
                MotorCommandConfig.velocity_kd if velocity_kd is None else velocity_kd,
                *MotorSaturation.kd,
            ),
            torque_ff_nm=MotorCommandConfig.torque_ff_nm,
        )

    @staticmethod
    def _build_signals(data_left, data_right) -> tuple[SensorSignal, SensorSignal]:
        """Turn a pair of IMU readings into raw sensor signals.

        Each limb keeps its own IMU timestamp; the row is stamped with the
        left one.

        The angle is the ``y`` component, not ``z``. ``z`` is yaw about the
        world vertical: it gimbal-locks at 90 degrees of flexion, drifts with
        the magnetometer disabled, and in a bench recording ratcheted through
        703 degrees of movements that all returned to neutral, taking a single
        108-degree step between samples. ``y`` is the thigh's tilt against
        gravity -- it tracked the same session cleanly, resting near -80
        degrees and rising to about 0 at full flexion with no wrapping, and
        its derivative matches the gyro at |r| > 0.91 on both legs.

        The gyro sign is per leg because ``gyro.z`` is read in the sensor's
        frame while the angle is resolved against gravity; see IMUMounting.

        :param data_left: Reading from the left IMU.
        :param data_right: Reading from the right IMU.
        :return: ``(left, right)`` raw signals.
        :rtype: tuple[SensorSignal, SensorSignal]
        """
        return (
            SensorSignal(
                angle_rad=data_left.quat.to_euler(seq="xyz").y,
                velocity_rad_per_sec=(
                    IMUMounting.gyro_sign_left * data_left.device_data.gyro.z
                ),
                timestamp=data_left.timestamp,
            ),
            SensorSignal(
                angle_rad=data_right.quat.to_euler(seq="xyz").y,
                velocity_rad_per_sec=(
                    IMUMounting.gyro_sign_right * data_right.device_data.gyro.z
                ),
                timestamp=data_right.timestamp,
            ),
        )

    def _record_sensors_only(self) -> None:
        """Record one row in a state where the controller does not run.

        Used for standby and pretensioning, so the recording covers the whole
        process rather than just the sessions: the file shows what the sensors
        saw between runs, at one spacing throughout.

        The controller is deliberately NOT stepped here. Doing so would let its
        SOGI-FLL and gait state evolve while idle and change how the next
        session begins, which is a control change rather than a recording one.
        The filtered and command columns are therefore NaN on these rows --
        nothing computed them, and repeating the last live value would read as
        output the controller never produced. ``exosuit_state`` says which
        state the row came from.

        :return: None
        """
        if not self.config.record:
            return

        data_left = self.imu_left.get_data()
        data_right = self.imu_right.get_data()
        if data_left is None or data_right is None:
            return

        raw_left, raw_right = self._build_signals(
            data_left=data_left, data_right=data_right
        )
        not_computed_left = SensorSignal(
            timestamp=raw_left.timestamp,
            angle_rad=math.nan,
            velocity_rad_per_sec=math.nan,
        )
        not_computed_right = SensorSignal(
            timestamp=raw_right.timestamp,
            angle_rad=math.nan,
            velocity_rad_per_sec=math.nan,
        )

        self._record_sample(
            raw=(raw_left, raw_right),
            filtered=(not_computed_left, not_computed_right),
            commands=(math.nan, math.nan),
        )

    def _record_sample(
        self,
        raw: tuple[SensorSignal, SensorSignal],
        filtered: tuple[SensorSignal, SensorSignal],
        commands: tuple[float, float],
        references: tuple[float, float] = (math.nan, math.nan),
        states: tuple[MotorTelemetry, MotorTelemetry] | None = None,
    ) -> None:
        """Buffer one row of this process's recording.

        Rows are held in memory and written once, at shutdown, so the control
        loop never touches the disk and one run produces one file.

        What the motor reports -- torque, speed, position and its fault code --
        is read from the transport's feedback cache; see ``_read_motor_state``.
        The control path passes the readings it already took rather than let
        this method take them again, so the row describes the same tick the
        command was computed from instead of one a few hundred microseconds
        later.

        Torque is the MIT feedback field decoded against the motor profile's
        torque range, so it is in N*m despite motor-python calling it
        ``current_amps``. A Nm/kg column would still need the subject's mass,
        which belongs in a config rather than a guess here.

        :param raw: ``(left, right)`` raw signals, before preprocessing.
        :param filtered: ``(left, right)`` preprocessed signals, or NaN values
            on rows where the controller did not run.
        :param commands: ``(left, right)`` commanded motor velocities in rad/s,
            NaN when idle.
        :param references: ``(left, right)`` motor position references in
            radians, NaN on rows where the controller did not run.
        :param states: ``(left, right)`` motor telemetry for this tick. Read
            here when not supplied, which is what the sensor-only path does.
        :return: None
        """
        not_measured = math.nan
        raw_left, raw_right = raw
        filtered_left, filtered_right = filtered
        command_left, command_right = commands
        reference_left, reference_right = references
        if states is None:
            states = (
                self._read_motor_state(self.motor_left),
                self._read_motor_state(self.motor_right),
            )
        left_state, right_state = states

        self.csv_writer.append_data(
            RecordData(
                timestamp=raw_left.timestamp if raw_left.timestamp else not_measured,
                raw_signal_left=raw_left,
                filtered_signal_left=filtered_left,
                raw_signal_right=raw_right,
                filtered_signal_right=filtered_right,
                motor_torque_nm_left=left_state.torque_nm,
                motor_speed_rad_per_sec_left=left_state.velocity_rad_per_sec,
                motor_position_rad_left=left_state.position_rad,
                motor_torque_nm_right=right_state.torque_nm,
                motor_speed_rad_per_sec_right=right_state.velocity_rad_per_sec,
                motor_position_rad_right=right_state.position_rad,
                motor_command_left=command_left,
                motor_command_right=command_right,
                motor_reference_left=reference_left,
                motor_reference_right=reference_right,
                operation_switch=self._operation_switch,
                tension_switch=self._tension_switch,
                motor_error_left=left_state.error_code,
                motor_error_right=right_state.error_code,
                baseline_offset_rad_left=self.controller_left.baseline_offset_rad,
                baseline_offset_rad_right=self.controller_right.baseline_offset_rad,
            )
        )

    def _start_recording(self) -> None:
        """Open the run's recording file, if recording is enabled.

        One file per run of the process. Rows stream into it as they are
        produced rather than being buffered: the exosuit records continuously,
        standby included, so buffering would grow by roughly 13 MB per minute
        at 100 Hz and a crash would take the whole recording with it.

        :return: None
        """
        if not self.config.record:
            return

        try:
            self.csv_writer.start_streaming()
        except OSError as err:
            logger.error(f"Could not open a recording file: '{err}'.")

    def _save_recording(self) -> None:
        """Close the run's recording, once, at shutdown.

        A failure here must not take the exosuit down with it -- by the time
        this runs the loop has stopped and the motors are idle, so the
        recording is the least important thing happening.

        :return: None
        """
        if self.csv_writer.is_streaming:
            try:
                self.csv_writer.stop_streaming()
            except OSError as err:
                logger.error(f"Could not close the recording: '{err}'.")
            return

        # Not streaming: either recording is off, or the file could not be
        # opened and rows fell back to the buffer.
        if not self.csv_writer.rows:
            return

        try:
            self.csv_writer.save_data()
        except OSError as err:
            logger.error(f"Could not save the recording: '{err}'.")
        finally:
            self.csv_writer.reset()

    def _pretension(self) -> None:
        """Execute one iteration of the pre-tensioning loop.

        Ported from ``motor_control.py`` (Subsystem3): each leg low-pass
        filters its own torque feedback, rectifies it, and runs the four-state
        chart in ``LegTensioner``. Pre-tensioning ends once both legs reach
        DISABLE -- either because the wearer released the switch, or because
        the torque threshold was met and the STOP hold has elapsed.

        :return: None
        """
        if not self._tension_switch:
            # Releasing the switch ends pre-tensioning whatever the charts are
            # doing -- on the rig the whole subsystem is gated by this switch.
            # Stopping explicitly rather than relying on a chart transition
            # also covers a tap too short for this thread to have armed the
            # charts at all, which would otherwise strand them in RESET with
            # no exit and freeze the loop here for good.
            self._release_motors()
            logger.info("State change: pretensioning -> standby")
            self._reset_tensioners()
            self._status = ExosuitStates.STANDBY
            return

        if self._tensioner_left.finished and self._tensioner_right.finished:
            # Both charts are terminal and the motors are already stopped, so
            # the pull is over. Hold here until the switch is released, and do
            # not poll: a status request is byte-identical to the MIT enable
            # frame, so reading a stopped motor would re-energise it.
            #
            # DISABLE is terminal on the rig too. Re-arming here instead --
            # resetting the charts and returning to standby while the switch
            # is still held -- let the handler send us straight back into
            # PRETENSIONING, pulling the tendon tighter on every cycle.
            return

        time_difference = self._elapsed_since_last_tick()
        on_off = int(self._tension_switch)

        for motor, tensioner in (
            (self.motor_left, self._tensioner_left),
            (self.motor_right, self._tensioner_right),
        ):
            torque_nm = self._read_torque(motor)
            if torque_nm is None:
                # Without feedback the chart is blind to its own threshold,
                # and pulling a tendon with no way to know when to stop is the
                # one thing this loop must not do. Abort rather than guess:
                # the placeholder this replaced reported success instead.
                logger.error("No torque feedback from a motor. Pre-tensioning aborted.")
                self._abort_pretensioning()
                return

            velocity_rad_per_sec, enable = tensioner.step(
                torque_nm, on_off, time_difference
            )

            if enable > 0:
                self._command_velocity(
                    motor,
                    velocity_rad_per_sec,
                    velocity_kd=TensionConfig.mit_velocity_kd,
                )
            elif tensioner.enable_changed:
                # Only on the transition. The rig fires its enable/disable
                # frames on edges, and the CAN stop() blocks for 60 ms of
                # settling -- six ticks at 100 Hz. Calling it every lap while
                # the other leg is still pulling would stall the loop and feed
                # a wrong dt to the still-running leg's filter and hold timer.
                if tensioner.finished:
                    self._zero_encoder(motor)
                motor.stop()

    def _zero_encoder(self, motor) -> None:
        """Make the tensioned position the motor's new zero.

        Sent on STOP -> DISABLE, where the rig sends it: the motor has held the
        tensioned position for the STOP hold and is still in motor mode, with
        the disable frame about to follow.

        The position loop reads the motor's own position, and the controller's
        reference is written about a zero that means "leg upright, tendon
        taut". Without this the reference is measured against wherever the
        spool happened to finish -- 8.19 rad into the left and 13.47 into the
        right on the 2026-09-21 run, the latter past the +/-12.5 rad the MIT
        position field encodes, so it had already wrapped before the assist
        began. Zeroing here gives the assist the full field either side of the
        tensioned point, which the rig notes is enough for any locomotion mode
        including stairs.

        A motor that cannot be zeroed is logged and left alone rather than
        stopping the session: pre-tensioning itself has succeeded by this
        point, and the unwrapping in the position loop still tracks the shaft.

        :param motor: The motor whose encoder to re-zero.
        :return: None
        """
        zero_position = getattr(motor, "zero_position", None)
        if zero_position is None:
            return
        try:
            zero_position()
        except Exception as err:
            logger.error(f"Could not zero a motor's encoder: '{err}'.")

    def _sleep_until_due(self, previous_due: float) -> float:
        """Sleep until the next tick falls due, and say when that was.

        Sleeping a whole period after the work makes every iteration take the
        period *plus* however long the work took, so the loop can never reach
        its configured rate: a bench run measured 78.6 Hz against the 100 Hz
        configured, a median period of 11.04 ms being 10 ms of sleep and 1.04
        ms of work. Sleeping only the remainder holds the cadence and stops
        the error accumulating across ticks.

        The rate is not cosmetic. ``BasicConfig(frequency=...)`` is handed the
        configured value, and hip-controller derives its notches and baseline
        window from it, so a loop that quietly runs a fifth slower than it
        claims mistunes the whole filter chain.

        After a tick that ran long the schedule restarts from now rather than
        from when it should have been. Catching up would fire a burst of
        back-to-back iterations, which is the opposite of what a control loop
        wants after it has already fallen behind.

        :param previous_due: When the tick that just ran was due.
        :return: When the next tick falls due.
        :rtype: float
        """
        next_due = previous_due + 1 / self.config.frequency
        remaining = next_due - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
            return next_due
        return time.monotonic()

    def _elapsed_since_last_control_tick(self) -> float:
        """Return the time since the previous assist tick, in seconds.

        The position loop's filter is rate dependent in the same way the
        tensioning chart's is, and this loop does not keep its nominal period:
        a bench run measured a median of 10.11 ms against a 10 ms target, a
        95th percentile of 15.18 ms and a worst case of 217.6 ms. The loop
        clamps the step it actually uses; this reports the real elapsed time
        and lets it decide.

        :return: Seconds since the previous tick, or the nominal period on the
            first tick of a session.
        :rtype: float
        """
        now = time.monotonic()
        previous, self._last_control_tick = self._last_control_tick, now
        if previous is None:
            return 1 / self.config.frequency
        return now - previous

    def _elapsed_since_last_tick(self) -> float:
        """Return the time since the previous pre-tensioning tick, in seconds.

        The chart's low-pass filter and its STOP hold are both rate dependent,
        and this loop does not keep its nominal period: a bench run measured a
        median of 11.6 ms against a 10 ms target, a 95th percentile of 23.8 ms
        and a worst case of 229 ms, with 14% of ticks over budget. Telling the
        filter every step was 10 ms made it lag, and stretched a "1 second"
        hold well past a second of wall clock.

        The step is capped even so. A stall would otherwise hand a 25 rad/s
        filter a step of hundreds of milliseconds and rail it, which is the
        failure the SOGI dt clamp exists for upstream; a late tick should cost
        accuracy, not stability.

        :return: Seconds since the previous tick, clamped, or the nominal
            period on the first tick of a session.
        :rtype: float
        """
        nominal = 1 / self.config.frequency
        now = time.monotonic()
        previous, self._last_pretension_tick = self._last_pretension_tick, now
        if previous is None:
            return nominal
        return min(now - previous, TensionConfig.max_time_step_periods * nominal)

    def _read_motor_state(self, motor) -> MotorTelemetry:
        """Return what the motor reports, in this package's units.

        Read from the transport's feedback cache rather than by requesting it:
        a request is a blocking round trip whose frame also re-enters MIT mode
        (see _read_torque), and every MIT command already draws a feedback
        frame, so while a motor is driven its telemetry is in hand.

        Recording this is what turns "the motors did not move" into an
        observation. Until now every motor column held NaN, so a session could
        show a healthy command stream and say nothing at all about whether the
        motor accepted it, produced torque, or reported a fault.

        Units follow the rest of this package rather than the wire: position
        and velocity in radians, torque in N*m. On the MIT path the transport
        decodes its ``current_amps`` field against the profile's torque range,
        so it is a torque despite the name.

        :param motor: The motor to read.
        :return: Its reported state, all NaN if nothing could be read.
        :rtype: MotorTelemetry
        """
        feedback = getattr(motor, "_last_feedback", None)
        if feedback is None:
            return MotorTelemetry(math.nan, math.nan, math.nan, math.nan)

        erpm_to_rad_s = getattr(motor, "_erpm_to_rad_s", None)
        speed = float(getattr(feedback, "speed_erpm", math.nan))
        return MotorTelemetry(
            position_rad=math.radians(
                float(getattr(feedback, "position_degrees", math.nan))
            ),
            velocity_rad_per_sec=(
                erpm_to_rad_s(speed) if erpm_to_rad_s is not None else math.nan
            ),
            torque_nm=float(getattr(feedback, "current_amps", math.nan)),
            error_code=float(getattr(feedback, "error_code", math.nan)),
        )

    def _read_torque(self, motor) -> float | None:
        """Return this leg's torque in N*m, preferring feedback already in hand.

        ``get_current()`` is not a read: it sends a status request and blocks
        on the reply for up to half a second. Worse, that request frame is
        byte-identical to the MIT enable frame -- the CubeMars manual uses the
        same frame for both -- so polling twice a tick interleaves ~170
        enter-motor-mode frames a second with the keep-alive thread's velocity
        commands. That is the likely source of the juddering seen on the first
        bench pull, and of the loop stutter measured alongside it.

        None of it is necessary while the motor is being driven: every MIT
        command draws a feedback frame, which the transport already parses and
        caches. Reading that cache costs nothing and disturbs nothing. The
        blocking path stays as a fallback for when the cache has gone stale --
        a motor that has stopped answering -- which is exactly when a real
        request is worth its cost.

        :param motor: The motor to read.
        :return: Torque in N*m, or None if no feedback can be obtained.
        :rtype: float or None
        """
        feedback = getattr(motor, "_last_feedback", None)
        if feedback is not None:
            age = time.monotonic() - getattr(motor, "_last_feedback_monotonic", 0.0)
            if age <= TensionConfig.torque_staleness_s:
                return feedback.current_amps
        return motor.get_current()

    def _release_motors(self) -> None:
        """Stop both motors, releasing the tendons.

        A motor fault must not take the loop down with it, so each stop is
        guarded: the other leg still needs releasing either way.

        :return: None
        """
        for motor in (self.motor_left, self.motor_right):
            try:
                motor.stop()
            except Exception as err:
                logger.error(f"Could not stop a motor: '{err}'.")

    def _abort_pretensioning(self) -> None:
        """Stop both motors and leave pre-tensioning without having tensioned.

        :return: None
        """
        self._release_motors()
        self._reset_tensioners()
        self._status = ExosuitStates.STANDBY

    def _reset_position_loops(self) -> None:
        """Clear both position loops, ready for the next assist session.

        Called when the assist stops, so the next one starts from where the
        tendon actually is rather than carrying this session's integrator,
        filter and accumulated unwrap into it.

        :return: None
        """
        self._position_loop_left.reset()
        self._position_loop_right.reset()
        # Including the give-up latch: a leg written off in one session gets a
        # fresh chance in the next rather than staying dead until a restart.
        self._watchdog_left.reset()
        self._watchdog_right.reset()
        self._reported_give_up.clear()
        # The next session measures its first step from its own start, not
        # from whenever this one happened to end.
        self._last_control_tick = None

    def _reset_tensioners(self) -> None:
        """Return both tensioning charts to RESET for the next session.

        :return: None
        """
        self._tensioner_left.reset()
        self._tensioner_right.reset()
        # The next session measures its first step from its own start, not
        # from whenever this one happened to end.
        self._last_pretension_tick = None

    def _initialize_imus(self) -> bool:
        """Initialize IMUs.

        :return: True if successful, False otherwise
        """
        left_init: bool = False
        right_init: bool = False

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            sensor_managers = IMUFactory.detect_and_create(
                free_threading=True,
                log_data=False,
                create_mock=self.config.mock_devices or self.config.test_gpio,
            )
        detected_imus = len(sensor_managers)
        if detected_imus < 2:
            logger.error(f"Wrong number of IMUs detected: {detected_imus} < 2")
        for idx in range(
            detected_imus
        ):  # Match each leg with the IMU according to IMUConfig
            manager = sensor_managers[idx]
            if (
                not left_init
                and manager.i2c_id == self._imu_cfg.left_leg_bus
                and manager.imu_descriptor == self._imu_cfg.left_leg_descr
            ):
                self.imu_left = manager
                left_init = True
                continue
            if (
                manager.i2c_id == self._imu_cfg.right_leg_bus
                and manager.imu_descriptor == self._imu_cfg.right_leg_descr
            ):
                self.imu_right = manager
                right_init = True
        if not left_init:
            logger.error("No detected IMUs matches left leg config")
        if not right_init:
            logger.error("No detected IMUs matches right leg config")
        return left_init and right_init

    def _initialize_motors(self) -> bool:
        """Initialize Motors.

        :return: True if successful, False otherwise
        """
        try:
            communication_status = True
            if not self.motor_left.check_communication():
                logger.error("Left Motor not responding. Check power and connections.")
                communication_status = False
            if not self.motor_right.check_communication():
                logger.error("Right Motor not responding. Check power and connections.")
                communication_status = False
            return communication_status
        except Exception as err:
            logger.error(f"Exosuit exception: '{err}'. Check Motor connections.")
            return False

    def _get_mode(
        self, switch_1: SwitchStates, switch_2: SwitchStates
    ) -> InclinationModes | None:
        """Get inclination mode based on the current switch states. Return None if unrecognized.

        :param switch_1: state of the first channel of the switch.
        :param switch_2: state of the second channel of the switch.
        :return: the matched inclination mode, or None if unrecognized.
        """
        for mode, state in MODE_SWITCH_LOGIC.items():
            if switch_1 == state.switch_1 and switch_2 == state.switch_2:
                return mode

        return None
