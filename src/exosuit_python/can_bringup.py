"""Bring the CAN interface up before anything tries to talk over it.

Until now this package assumed ``can0`` was already configured, which meant
``sudo ./setup_can.sh`` from the motor-module repository had to be remembered
after every power cycle. Forgetting it produces "Left Motor not responding.
Check power and connections." -- a message that sends you to the wiring for a
fault that is one command away.

The bring-up is **idempotent by design**: an interface already up at the right
bitrate is left strictly alone. Reconfiguring a working bus is not free --
taking the link down and up resets the controller and can drop a motor into
BUS-OFF -- so the check exists to avoid doing that, not merely to save time.

What it does not do is reload the ``mttcan`` kernel module. That is the heavier
recovery for a controller whose error counters have latched, it belongs to
``motor_python.can_utils.reset_can_interface``, and it is the wrong thing to do
unasked at startup.
"""

import shutil
import subprocess

from loguru import logger

from exosuit_python.definitions import CanConfig


def _run(command: list[str], timeout: float) -> subprocess.CompletedProcess | None:
    """Run a command, returning None if it could not be run at all.

    :param command: Argument vector.
    :param timeout: Seconds to wait before giving up.
    :return: The finished process, or None if it could not run.
    """
    try:
        # check=False: a non-zero exit is information here, not an exception.
        # The caller reads returncode and reports it with the interface's name.
        return subprocess.run(
            command, capture_output=True, text=True, timeout=timeout, check=False
        )
    except (OSError, subprocess.SubprocessError) as err:
        logger.debug(f"Could not run {' '.join(command)}: '{err}'.")
        return None


def read_interface(config: CanConfig) -> tuple[bool, bool, int | None]:
    """Report what the kernel currently thinks of the interface.

    :param config: Which interface to look at, and how.
    :return: ``(exists, is_up, bitrate)``; bitrate is None when unreadable.
    :rtype: tuple[bool, bool, int | None]
    """
    result = _run(
        ["ip", "-details", "link", "show", config.interface], config.command_timeout_s
    )
    if result is None or result.returncode != 0:
        return False, False, None

    output = result.stdout
    # The flags live in angle brackets on the first line. "UP" also appears in
    # "state UP" further along, but the flag is the authoritative one -- an
    # interface can carry the UP flag while its state reads UNKNOWN, which is
    # normal for CAN.
    first_line = output.splitlines()[0] if output else ""
    is_up = "UP" in first_line.split("<")[-1].split(">")[0].split(",")

    bitrate = None
    fields = output.split()
    if "bitrate" in fields:
        try:
            bitrate = int(fields[fields.index("bitrate") + 1])
        except (IndexError, ValueError):
            bitrate = None
    return True, is_up, bitrate


def ensure_can_interface(config: CanConfig | None = None) -> bool:
    """Make sure the CAN interface is up at the configured bitrate.

    :param config: Interface settings. Defaults to ``CanConfig()``.
    :return: True if the interface is usable afterwards.
    :rtype: bool
    """
    config = config if config is not None else CanConfig()

    if shutil.which("ip") is None:
        logger.debug(
            "No 'ip' command, so this is not a Linux host. Skipping CAN setup."
        )
        return False

    exists, is_up, bitrate = read_interface(config)
    if not exists:
        logger.error(
            f"No CAN interface '{config.interface}'. The mttcan kernel module is "
            f"probably not loaded; run setup_can.sh once, which modprobes it."
        )
        return False

    if is_up and bitrate == config.bitrate:
        logger.info(
            f"CAN interface '{config.interface}' is already up at "
            f"{config.bitrate} bps. Leaving it alone."
        )
        return True

    reason = "down" if not is_up else f"at {bitrate} bps rather than {config.bitrate}"
    logger.warning(f"CAN interface '{config.interface}' is {reason}. Bringing it up.")
    return _bring_up(config)


def _bring_up(config: CanConfig) -> bool:
    """Configure and raise the interface.

    ``sudo -n`` throughout: a control process must not stop at a password
    prompt, so a host without passwordless sudo fails immediately and says so
    rather than hanging until something times out.

    :param config: Interface settings.
    :return: True if the interface came up at the right bitrate.
    :rtype: bool
    """
    commands = [
        # Down first: "up type can bitrate ..." is rejected on an interface
        # that is already up, which is the case when only the bitrate is wrong.
        ["sudo", "-n", "ip", "link", "set", config.interface, "down"],
        # berr-reporting and restart-ms are what let the controller recover
        # from BUS-OFF by itself instead of going permanently silent after an
        # unacknowledged frame.
        [
            "sudo",
            "-n",
            "ip",
            "link",
            "set",
            config.interface,
            "up",
            "type",
            "can",
            "bitrate",
            str(config.bitrate),
            "berr-reporting",
            "on",
            "restart-ms",
            str(config.restart_ms),
        ],
        # The default queue of 10 is too shallow for a 100 Hz loop driving two
        # motors.
        [
            "sudo",
            "-n",
            "ip",
            "link",
            "set",
            config.interface,
            "txqueuelen",
            str(config.tx_queue_length),
        ],
    ]

    for command in commands:
        result = _run(command, config.command_timeout_s)
        if result is None or result.returncode != 0:
            detail = (result.stderr or result.stdout).strip() if result else "not run"
            logger.error(
                f"Could not bring up '{config.interface}': '{detail}'. "
                f"Run scripts/allow_can_bringup_without_password.sh once to fix "
                f"this permanently, or setup_can.sh with sudo for just this boot."
            )
            return False

    _, is_up, bitrate = read_interface(config)
    if not (is_up and bitrate == config.bitrate):
        logger.error(
            f"'{config.interface}' did not come up as asked: up={is_up}, "
            f"bitrate={bitrate}."
        )
        return False

    logger.success(f"CAN interface '{config.interface}' up at {config.bitrate} bps.")
    return True
