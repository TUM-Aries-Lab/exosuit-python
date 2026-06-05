"""Mock GPIO for CI testing."""

import threading
import time
from collections.abc import Callable

from exosuit_python.definitions import THREAD_JOIN_TIMEOUT


class MockGPIO:
    """Mock GPIO for CI testing."""

    OUT = 0
    IN = 1
    RISING = 31
    FALLING = 32
    BOARD = 10
    PUD_UP = 22

    def __init__(self) -> None:
        self.gpio_running: bool = False
        self._threads: list[threading.Thread] = []
        self._detected_events: dict[int, int | None] = {}

    def setmode(self, mode: int) -> None:
        """Set mode."""
        pass

    def setup(self, channels: int, direction: int, pull_up_down: int = 0) -> None:
        """Set up GPIO."""
        self.gpio_running = True

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

    def simulate_switch(self, channel: int, event: int) -> None:
        """Simulate switches for event detection."""
        self._detected_events[channel] = event

    def _loop_event_detect_thread(
        self,
        channel: int,
        edge: int,
        callback: Callable | None = None,
        bouncetime: int | None = None,
        polltime: float = 0.2,
    ) -> None:
        while self.gpio_running:
            # Check if an event was simulated for this channel
            if (
                channel in self._detected_events
                and self._detected_events[channel] == edge
            ):
                # Call the callback
                if callback:
                    callback(channel)
                # Clear the event to prevent duplicate triggers
                self._detected_events[channel] = None

            time.sleep(polltime)
