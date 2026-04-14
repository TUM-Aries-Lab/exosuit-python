# exosuit-python

Top-level integration package for the ARIES Lab (IBRS, TUM) hip flexion exosuit. Wires together IMU sensing, control algorithms, and motor communication into one real-time system running on a Jetson Orin Nano at 100 Hz.

**PyPI:** exosuit-python | **GitHub:** TUM-Aries-Lab/exosuit-python | **Current version:** 0.0.3 (controller branch) | **Python:** 3.11+

## Project Structure

```
src/exosuit_python/
├── __init__.py
├── __main__.py              # Entry point — starts the exosuit
├── definitions.py           # ExosuitConfig, ConfigTension, constants
├── exosuit.py               # Exosuit class — main orchestration
├── csv_writer.py            # Records sensor + motor data to CSV
└── utils.py                 # Logging setup, unit conversion (rad/s → eRPM)
```

Uses src-layout. Package metadata and dependencies in `pyproject.toml`.
Tests in `tests/` at root level.

## Branches

- **main** (v0.0.2) — IMU integrated, placeholder controller (no real control)
- **controller** (v0.0.3) — Full integration with WalkOnController, CSV recording, tensioning logic ← most complete, focus of this doc
- **imu** (v0.0.2) — Early skeleton
- **normalization-and-centering** — Local development

All branches will eventually merge into main.

## Data Flow (One Control Loop Iteration)

```
IMU Hardware (BNO055 on I2C)
    │
    ├─► imu_right.get_data() → IMUData
    │       ├─► .quat.to_euler("xyz").z  → angle_rad
    │       └─► .device_data.gyro.z      → velocity_rad_per_sec
    │
    ├─► SensorSignal(angle_rad, velocity_rad_per_sec)
    │
    ├─► controller_right.step(timestamp, signal)
    │       ├─► HighLevelController → gait_phase
    │       ├─► AmplitudeModulation → amplitude
    │       └─► MidLevelController → motor_command (rad/s)
    │
    ├─► convert_rad_per_sec_to_rpm(motor_command) → eRPM
    │
    └─► motor_right.set_velocity(eRPM)

(same for left leg)

    └─► time.sleep(1/frequency)
```

## Hardware Setup

- **Hip IMU:** BNO055 on I2C bus 1 (`imu_hip`)
- **Leg IMUs:** BNO055 on I2C bus 7 (`imu_left` index 0, `imu_right` index 1)
- **Left motor:** CubeMars AK60-6 V3 on `/dev/ttyTHS1` (`motor_left`)
- **Right motor:** Currently a placeholder (not yet connected)
- **Platform:** Jetson Orin Nano
- **Loop frequency:** 100 Hz

## Lifecycle

1. `__main__.py` → `setup_logger()` → `ExosuitConfig(frequency=100)` → `Exosuit(config)`
2. `__init__()` → creates controllers (WalkOnController L/R), motor, detects IMUs via IMUFactory
3. `turn_on_exosuit_switch()` → starts IMU managers, starts control loop daemon thread
4. `_loop()` runs at 100 Hz until `KeyboardInterrupt`
5. `_cleanup()` → stops IMUs, closes motor, joins thread

## Configuration (definitions.py)

- `ExosuitConfig.frequency = 100` — Main loop rate (Hz)
- `ConfigTension.tensioning_velocity = 3` — Motor velocity during tensioning
- `ConfigTension.motor_torque_limit = 0.85` — Torque threshold for taut tendon detection
- `ConfigTension.tensioning_timeout = 1.0` — Post-tensioning wait (s)
- `THREAD_JOIN_TIMEOUT = 2.0` — Max wait when stopping threads

## Known Issues (important for reviews)

1. **Copy-paste bug in `_loop()`** — Left leg velocity reads from `imu_right` instead of `imu_left` in the `signal_left` block.
2. **Multiple `get_data()` calls per IMU per iteration** — timestamp, angle, velocity each call `get_data()` separately. If IMU updates between calls, data is inconsistent. Should capture once per leg per iteration.
3. **`time.sleep(1/frequency)` for loop timing** — Does not subtract computation time, so actual rate is always slower than 100 Hz.
4. **Right motor is a placeholder** — `self.motor_right = ["Motor right."]` with `.append()` accumulates strings indefinitely.
5. **Tensioning incomplete** — Only sets velocity, missing the full state machine (vel → torque threshold → stop → wait → disable).
6. **No signal filtering** — Raw IMU data goes directly to controller without drift removal, SOGI+FLL, or any filtering.
7. **PID not in the loop** — Motor command goes directly to `set_velocity()` without PID feedback from motor position.

## Dependencies (auto-installed via pyproject.toml)

- `hip-controller` — Control algorithm
- `imu-python` — IMU communication and orientation
- `motor-python` — CubeMars motor communication
- `loguru`, `numpy`, `pandas`

## How to Run

```bash
pip install -e .
python -m exosuit_python       # Start the exosuit (requires hardware)
pytest
ruff check .
```

## Conventions

See `.claude/skills/code-review.md` for the full coding conventions checklist.

## Restrictions

- **Never modify motor safety limits** — these are in `motor-python`, not here
- **Never change I2C bus assignments** without hardware verification
- **Never hardcode hardware-specific paths** (serial ports, I2C addresses)
- **Never bypass the cleanup sequence** — IMUs must stop, motor must close
- **Never push directly to main** — always use pull requests
- **Always test with recorded data before testing on hardware** — use CSV playback via `hip-controller`'s `csv_player.py`
- Always run `ruff check .` and `pytest` before considering a task complete
