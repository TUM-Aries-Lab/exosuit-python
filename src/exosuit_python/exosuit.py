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
    InclinationModes,
    MotorCommandConfig,
    SwitchStates,
    TensionConfig,
)
from exosuit_python.gpio import MockGPIO
from exosuit_python.motor import MockMotor
from exosuit_python.tensioning import LegTensioner


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
        # the controller. Per-limb wiring reversal comes from the config's
        # left_limb_reverse / right_limb_reverse defaults, which match the
        # False/True this used to pass positionally.
        self.controller_config = BasicConfig(
            frequency=int(config.frequency), filtered=False
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

        self.motor_left: CubeMarsAK806v2CAN | MockMotor
        self.motor_right: CubeMarsAK806v2CAN | MockMotor

        if self.config.mock_devices or self.config.test_gpio:
            self.motor_left = MockMotor()
            self.motor_right = MockMotor()
        else:
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

        # One tensioning chart per leg. The legs are mirrored, so they pull in
        # opposite directions.
        self._tensioner_left = LegTensioner(
            TensionConfig.tensioning_velocity_left_rad_per_sec
        )
        self._tensioner_right = LegTensioner(
            TensionConfig.tensioning_velocity_right_rad_per_sec
        )

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
            self.gpio.setmode(self.gpio.BOARD)
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
        while self._status != ExosuitStates.STOPPED:
            while self._status == ExosuitStates.RUNNING:
                try:
                    self._control()
                except TypeError as err:
                    logger.error(f"Failed getting data from the IMU: '{err}'.")
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

                time.sleep(1 / self.config.frequency)

            # Session over. _control() cannot deliver the falling edge itself:
            # it stops being called the moment the status leaves RUNNING. Without
            # this the trigger stays high inside the controller, the next session
            # produces no rising edge, and no baseline is ever taken again --
            # baseline removal would work on the first run of the process and
            # silently never again. Idempotent: the controller acts on edges, so
            # repeating it while idle does nothing.
            self._set_baseline_removal_trigger(False)

            # Pretensioning ticks at the control rate, like every other lap, so
            # the recording keeps one row spacing for the whole run. The chart
            # and its torque low-pass filter are stepped every tick for the
            # same reason the rig does: both are rate-dependent, and the STOP
            # hold is counted in ticks rather than slept through.
            while self._status == ExosuitStates.PRETENSIONING:
                try:
                    self._pretension()
                    self._record_sensors_only()
                except Exception as err:
                    logger.error(f"Exosuit control loop exception: '{err}'.")

                time.sleep(1 / self.config.frequency)

            # Keep recording while idle, at the same rate, so one run produces
            # one continuous file at one spacing. Written once, at shutdown.
            while self._status == ExosuitStates.STANDBY:
                try:
                    self._record_sensors_only()
                except Exception as err:
                    logger.error(f"Exosuit idle recording exception: '{err}'.")

                time.sleep(1 / self.config.frequency)

            # Reached only in states with no loop of their own (INITIALIZING),
            # and keeps this from becoming a busy spin.
            time.sleep(EXOSUIT_STANDBY_INTERVAL)

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
        command_right = self.controller_right.step(curr_signal=signal_right)
        command_left = self.controller_left.step(curr_signal=signal_left)

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
            )

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
        motor.set_mit_mode(
            pos_rad=0.0,
            vel_rad_s=velocity_rad_per_sec,
            kp=MotorCommandConfig.kp,
            kd=(MotorCommandConfig.velocity_kd if velocity_kd is None else velocity_kd),
            torque_ff_nm=MotorCommandConfig.torque_ff_nm,
        )

    @staticmethod
    def _build_signals(data_left, data_right) -> tuple[SensorSignal, SensorSignal]:
        """Turn a pair of IMU readings into raw sensor signals.

        Each limb keeps its own IMU timestamp; the row is stamped with the
        left one.

        :param data_left: Reading from the left IMU.
        :param data_right: Reading from the right IMU.
        :return: ``(left, right)`` raw signals.
        :rtype: tuple[SensorSignal, SensorSignal]
        """
        return (
            SensorSignal(
                angle_rad=data_left.quat.to_euler(seq="xyz").z,
                velocity_rad_per_sec=data_left.device_data.gyro.z,
                timestamp=data_left.timestamp,
            ),
            SensorSignal(
                angle_rad=data_right.quat.to_euler(seq="xyz").z,
                velocity_rad_per_sec=data_right.device_data.gyro.z,
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
    ) -> None:
        """Buffer one row of this process's recording.

        Rows are held in memory and written once, at shutdown, so the control
        loop never touches the disk and one run produces one file.

        Motor torque, speed and position are written as NaN -- not measured, as
        opposed to a zero, which would read as "the motor produced no torque".

        Position and speed *are* obtainable: ``CubeMarsAK606v3.get_status()``
        returns raw bytes that ``motor_python.MotorStatusParser`` decodes into
        ``position_degrees`` and ``speed_erpm``. They are not wired up here
        because ``get_status()`` is a blocking serial round-trip, and two of
        them per iteration inside the loop that drives the motors is a
        real-time risk worth measuring before taking it. ``MockMotor`` would
        also need the same call, which it does not have today.

        Torque is not measured by the motor at all: it reports current, so the
        Nm/kg column needs iq_current x Kt and the subject's mass -- two
        constants that belong in a config, not in a guess here.

        :param raw: ``(left, right)`` raw signals, before preprocessing.
        :param filtered: ``(left, right)`` preprocessed signals, or NaN values
            on rows where the controller did not run.
        :param commands: ``(left, right)`` commanded motor values, NaN when idle.
        :return: None
        """
        not_measured = math.nan
        raw_left, raw_right = raw
        filtered_left, filtered_right = filtered
        command_left, command_right = commands

        self.csv_writer.append_data(
            RecordData(
                timestamp=raw_left.timestamp if raw_left.timestamp else not_measured,
                raw_signal_left=raw_left,
                filtered_signal_left=filtered_left,
                raw_signal_right=raw_right,
                filtered_signal_right=filtered_right,
                motor_torque_nm_per_kg_left=not_measured,
                motor_speed_rad_per_sec_left=not_measured,
                motor_position_rad_left=not_measured,
                motor_torque_nm_per_kg_right=not_measured,
                motor_speed_rad_per_sec_right=not_measured,
                motor_position_rad_right=not_measured,
                motor_command_left=command_left,
                motor_command_right=command_right,
                operation_switch=self._operation_switch,
                tension_switch=self._tension_switch,
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
        time_difference = 1 / self.config.frequency
        on_off = int(self._tension_switch)

        for motor, tensioner in (
            (self.motor_left, self._tensioner_left),
            (self.motor_right, self._tensioner_right),
        ):
            torque_nm = motor.get_current()
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
                motor.stop()

        if self._tensioner_left.finished and self._tensioner_right.finished:
            logger.info("State change: pretensioning -> standby")
            self._reset_tensioners()
            self._status = ExosuitStates.STANDBY

    def _abort_pretensioning(self) -> None:
        """Stop both motors and leave pre-tensioning without having tensioned.

        :return: None
        """
        for motor in (self.motor_left, self.motor_right):
            try:
                motor.stop()
            except Exception as err:
                logger.error(f"Could not stop a motor: '{err}'.")
        self._reset_tensioners()
        self._status = ExosuitStates.STANDBY

    def _reset_tensioners(self) -> None:
        """Return both tensioning charts to RESET for the next session.

        :return: None
        """
        self._tensioner_left.reset()
        self._tensioner_right.reset()

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
                and manager.i2c_id == self.config.imu_cfg.left_leg_bus
                and manager.imu_descriptor == self.config.imu_cfg.left_leg_descr
            ):
                self.imu_left = manager
                left_init = True
                continue
            if (
                manager.i2c_id == self.config.imu_cfg.right_leg_bus
                and manager.imu_descriptor == self.config.imu_cfg.right_leg_descr
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
