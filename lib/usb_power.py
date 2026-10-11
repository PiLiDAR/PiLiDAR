'''
USB port power control for the LiDAR (lib/usb_power.py).

The STL27L LiDAR is powered over USB; its controller board is a Silicon Labs
CP210x UART bridge behind a per-port-power-switching hub. Cutting USB VBUS for
that port turns the LiDAR fully off while idle — this replaces the obsolete LD06
power relay, which was only needed because the LD06 was a bare UART board.

Uses [uhubctl](https://github.com/mvp/uhubctl); needs passwordless sudo for it
(the default Raspberry Pi OS ``pi`` user has ``NOPASSWD: ALL``). Settings live in
``config.json`` under ``USB_POWER``::

    "USB_POWER": {"ENABLE": true, "HUB": "1-1", "PORT": 1, "DELAY": 5}

This module reads ``config.json`` directly (not via :class:`lib.config.Config`)
so it can power the LiDAR on *before* ``Config.__init__`` resolves the serial
port — otherwise a powered-off LiDAR makes the port fall back to ``/dev/ttyS0``.
'''

import json
import logging
import os
import shutil
import subprocess
import time

logger = logging.getLogger(__name__)

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(_ROOT, "config.json")

# uhubctl lives in /usr/sbin, which is not on the default non-root PATH
_UHUBCTL_CANDIDATES = ("uhubctl", "/usr/sbin/uhubctl", "/usr/local/sbin/uhubctl")


def find_uhubctl():
    """Return the path to the uhubctl executable, or None."""
    for candidate in _UHUBCTL_CANDIDATES:
        if os.path.isabs(candidate):
            if os.path.exists(candidate):
                return candidate
        else:
            found = shutil.which(candidate)
            if found:
                return found
    return None


def load_settings(config_path=CONFIG_PATH):
    """Read the USB_POWER block + the LiDAR serial port from config.json."""
    with open(config_path) as f:
        cfg = json.load(f)

    usb = cfg.get("USB_POWER", {})
    lidar = cfg.get("LIDAR", {})
    device = lidar.get("DEVICE")
    device_port = lidar.get(device, {}).get("PORT", "/dev/ttyUSB0")

    return {
        "enabled": bool(usb.get("ENABLE", False)),
        "hub": usb.get("HUB", "1-1"),
        "port": usb.get("PORT", 1),
        "delay": usb.get("DELAY", 5),
        "device": device_port,
    }


def _switch(action, hub, port, delay=None):
    """Run ``uhubctl -a <action>`` on one port. Returns True on success."""
    exe = find_uhubctl()
    if exe is None:
        logger.warning("uhubctl not found; USB power switching unavailable")
        return False

    # -e (exact location) disables uhubctl's USB3 duality handling, so only the
    # target port on the target hub is switched (the xHCI root hub mirror is not).
    cmd = ["sudo", "-n", exe, "-l", str(hub), "-p", str(port), "-a", action, "-e"]
    if delay is not None:
        cmd += ["-d", str(delay)]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    except (subprocess.TimeoutExpired, OSError) as e:
        logger.warning("uhubctl %s failed: %s", action, e)
        return False
    if result.returncode != 0:
        logger.warning("uhubctl %s failed (rc=%d): %s", action, result.returncode, result.stderr.strip())
        return False
    return True


def _wait_for_device(path, timeout=15.0):
    """Wait until a device node appears (e.g. /dev/ttyUSB0 after power on)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(0.25)
    return False


def lidar_on(settings=None):
    """Power the LiDAR USB port on, wait for it to enumerate, and settle.

    Call this before constructing :class:`lib.config.Config` so the serial port
    resolves correctly. Returns True if the port was switched on (or the feature
    is disabled/no-op), False on failure.
    """
    s = settings or load_settings()
    if not s["enabled"]:
        return False
    if not _switch("on", s["hub"], s["port"]):
        return False
    if not _wait_for_device(s["device"]):
        logger.warning("LiDAR device %s did not appear after power on", s["device"])
        return False
    time.sleep(s["delay"])  # controller boot + motor spin-up
    logger.info("LiDAR USB power on (%s port %s)", s["hub"], s["port"])
    return True


def lidar_off(settings=None):
    """Cut power to the LiDAR USB port (fully off). Returns True on success."""
    s = settings or load_settings()
    if not s["enabled"]:
        return False
    ok = _switch("off", s["hub"], s["port"])
    if ok:
        logger.info("LiDAR USB power off (%s port %s)", s["hub"], s["port"])
    return ok
