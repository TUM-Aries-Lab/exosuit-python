"""Mock GPIO for CI testing."""

import threading
import time
from collections.abc import Callable

from exosuit_python.definitions import THREAD_JOIN_TIMEOUT


class MockGPIO:
    """Mock GPIO for CI testing."""

    LOW = 0
    HIGH = 1
    OUT = 0
    IN = 1
    RISING = 31
    FALLING = 32
    BOTH = 33
    BOARD = 10
    PUD_UP = 22

    def __init__(self) -> None:
        self.gpio_running: bool = False
        self._threads: list[threading.Thread] = []
        self._channel_states: dict[int, int] = {}  # Track current state (0=LOW, 1=HIGH)
        self._previous_states: dict[
            int, int | None
        ] = {}  # Track previous state for edge detection

    def setmode(self, mode: int) -> None:
        """Set mode."""
        pass

    def setup(self, channels: int, direction: int, pull_up_down: int = 0) -> None:
        """Set up GPIO."""
        self.gpio_running = True
        # Initialize channel state(s) when set up
        if isinstance(channels, int):
            self._channel_states[channels] = 0
            self._previous_states[channels] = None
        else:
            for ch in channels:
                self._channel_states[ch] = 0
                self._previous_states[ch] = None

    def add_event_detect(
        self,
        channel: int,
        edge: int,
        callback: Callable | None = None,
        bouncetime: int | None = None,
        polltime: float = 0.2,
    ) -> None:
        """Add detection for the given channel's event and call the callback function."""
        thread = threading.Thread(
            target=self._loop_event_detect_thread,
            kwargs={
                "channel": channel,
                "edge": edge,
                "callback": callback,
                "bouncetime": bouncetime,
                "polltime": polltime,
            },
            daemon=True,
        )
        self._threads.append(thread)
        thread.start()

    def cleanup(self):
        """Clean up GPIO."""
        self.gpio_running = False
        for thread in self._threads:
            thread.join(timeout=THREAD_JOIN_TIMEOUT)

    def simulate_switch(self, channel: int, state: int) -> None:
        """Simulate a switch state change (HIGH or LOW) to trigger edge detection."""
        self._channel_states[channel] = state

    def _loop_event_detect_thread(
        self,
        channel: int,
        edge: int,
        callback: Callable | None = None,
        bouncetime: int | None = None,
        polltime: float = 0.2,
    ) -> None:
        while self.gpio_running:
            current_state = self._channel_states.get(channel, 0)
            previous_state = self._previous_states.get(channel)

            # Detect edge transitions if we have a previous state
            if previous_state is not None:
                # RISING edge: LOW → HIGH
                if previous_state == self.LOW and current_state == self.HIGH:
                    if edge in {self.RISING, self.BOTH}:
                        if callback:
                            callback(channel)
                # FALLING edge: HIGH → LOW
                elif previous_state == self.HIGH and current_state == self.LOW:
                    if edge in {self.FALLING, self.BOTH}:
                        if callback:
                            callback(channel)

            # Update previous state for next iteration
            self._previous_states[channel] = current_state
            time.sleep(polltime)

    def input(self, channel) -> int:
        """Read the current state of a channel (HIGH or LOW)."""
        return self._channel_states.get(channel, 0)
