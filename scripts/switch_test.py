"""Read the four switch pins directly. No exosuit, no motors, no IMUs."""

import time

from loguru import logger

from exosuit_python.definitions import (
    MODE_SWITCH_1,
    MODE_SWITCH_2,
    MODE_SWITCH_LOGIC,
    OPERATION_SWITCH,
    TENSION_SWITCH,
    SwitchStates,
)

try:
    from Jetson import GPIO
except Exception:
    GPIO = None

POLL_INTERVAL = 0.05

SWITCH_PINS = {
    "OPERATION": OPERATION_SWITCH,
    "TENSION": TENSION_SWITCH,
    "MODE_1": MODE_SWITCH_1,
    "MODE_2": MODE_SWITCH_2,
}


def get_mode(states):
    """Return the inclination mode implied by the mode switch readings."""
    switch_1 = SwitchStates.ON if states["MODE_1"] else SwitchStates.OFF
    switch_2 = SwitchStates.ON if states["MODE_2"] else SwitchStates.OFF
    for mode, state in MODE_SWITCH_LOGIC.items():
        if switch_1 == state.switch_1 and switch_2 == state.switch_2:
            return mode.name
    return "unrecognized"


def main():
    """Poll the switch pins and log every change until interrupted."""
    if GPIO is None:
        logger.error("Jetson.GPIO unavailable. Run this on the Jetson.")
        return

    GPIO.setmode(GPIO.BOARD)
    for pin in SWITCH_PINS.values():
        GPIO.setup(pin, GPIO.IN)

    states = {name: GPIO.input(pin) for name, pin in SWITCH_PINS.items()}
    for name, pin in SWITCH_PINS.items():
        logger.info(f"{name:<10} pin {pin:<3} = {states[name]}")
    logger.info(f"mode: {get_mode(states)}")
    logger.info("Flip one switch at a time. Ctrl+C to stop.")

    try:
        while True:
            for name, pin in SWITCH_PINS.items():
                value = GPIO.input(pin)
                if value != states[name]:
                    states[name] = value
                    logger.success(f"{name:<10} pin {pin:<3} -> {value}")
                    if name.startswith("MODE"):
                        logger.info(f"mode: {get_mode(states)}")
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
