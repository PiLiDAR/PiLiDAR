'''
USB port power control for the LiDAR (lib/usb_power.py).

The STL27L is USB-powered: cutting USB VBUS while idle turns it fully off (no
spin, no power draw) — this replaces the obsolete LD06 power relay.

On the **Raspberry Pi 4** the USB-A power switching lives in the VL805 USB3
controller (USB bus 2), and *only whole-hub switching actually cuts VBUS*::

    sudo uhubctl -l 2 -a 0     # all USB-A ports off
    sudo uhubctl -l 2 -a 1     # on

Per-port (`-p`) and exact-location (`-e`) flags change the reported port state
and drop the USB *data* link, but do NOT remove power (the LiDAR keeps running).
Reference: https://raspberrypi.stackexchange.com/questions/5407#74906

Uses [uhubctl](https://github.com/mvp/uhubctl); needs passwordless sudo for it
(the default Raspberry Pi OS ``pi`` user has ``NOPASSWD: ALL``). Settings live in
``config.json``::

    "USB_POWER": {"ENABLE": true, "HUB": "2", "DELAY": 5}

This module reads ``config.json`` directly (not via :class:`lib.config.Config`)
so it can power the LiDAR on *before* ``Config.__init__`` resolves the serial
port — otherwise a powered-off LiDAR makes the port fall back to ``/dev/ttyS0``.
'''

import glob
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


def resolve_serial_device(port=None, by_id_glob=None):
    """Resolve the LiDAR serial device, preferring a stable by-id path.

    Power-cycling the USB port renumbers ``ttyUSB`` (0 -> 1 -> ...), so a raw
    ``/dev/ttyUSB0`` is not stable. If ``by_id_glob`` matches under
    ``/dev/serial/by-id`` the first match is returned, else ``port``.
    """
    if by_id_glob:
        matches = sorted(glob.glob(os.path.join("/dev/serial/by-id", by_id_glob)))
        if matches:
            return matches[0]
    return port or "/dev/ttyUSB0"


def load_settings(config_path=CONFIG_PATH):
    """Read the USB_POWER block + the LiDAR serial device from config.json."""
    with open(config_path) as f:
        cfg = json.load(f)

    usb = cfg.get("USB_POWER", {})
    lidar = cfg.get("LIDAR", {})
    device = lidar.get("DEVICE")
    device_cfg = lidar.get(device, {})

    return {
        "enabled": bool(usb.get("ENABLE", False)),
        "hub": usb.get("HUB", "2"),
        "delay": usb.get("DELAY", 5),
        # raw port + by-id glob; the device is resolved after power-on, since the
        # stable by-id link only exists while the LiDAR is powered
        "port": device_cfg.get("PORT", "/dev/ttyUSB0"),
        "port_by_id": device_cfg.get("PORT_BY_ID"),
    }


def _switch(action, hub, delay=None):
    """Run ``uhubctl -a <action>`` on a whole hub. Returns True on success."""
    exe = find_uhubctl()
    if exe is None:
        logger.warning("uhubctl not found; USB power switching unavailable")
        return False

    # Whole-hub only — no -p/-e, which do not switch VBUS on the Pi 4.
    cmd = ["sudo", "-n", exe, "-l", str(hub), "-a", action]
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
    """Wait until a device node appears (e.g. /dev/ttyUSB1 after power on)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(0.25)
    return False


def lidar_on(settings=None):
    """Power the LiDAR USB hub on, wait for it to enumerate, and settle.

    Call this before constructing :class:`lib.config.Config` so the serial port
    resolves correctly. The device path is resolved *after* power-on, since the
    stable ``/dev/serial/by-id`` link only exists while the LiDAR is powered.
    Returns True if switched on.
    """
    s = settings or load_settings()
    if not s["enabled"]:
        return False
    if not _switch("on", s["hub"]):
        return False
    # resolve the device now (by-id link exists only while powered)
    device = resolve_serial_device(
        s.get("port", "/dev/ttyUSB0"),
        s.get("port_by_id"),
    )
    if not _wait_for_device(device):
        logger.warning("LiDAR device %s did not appear after power on", device)
        return False
    time.sleep(s["delay"])  # controller boot + motor spin-up
    logger.info("LiDAR USB power on (hub %s, %s)", s["hub"], device)
    return True


def lidar_off(settings=None):
    """Cut power to the LiDAR USB hub (fully off). Returns True on success."""
    return hub_off(settings)


def hub_on(settings=None):
    """Power the USB hub on (no device wait) — e.g. to attach a USB stick.

    Unlike :func:`lidar_on` this does not wait for the LiDAR to enumerate; it is
    the plain "give me USB-A power" primitive (used by the button long-press).
    """
    s = settings or load_settings()
    if not s["enabled"]:
        return False
    ok = _switch("on", s["hub"])
    if ok:
        logger.info("USB hub %s power on", s["hub"])
    return ok


def hub_off(settings=None):
    """Cut power to the USB hub. Returns True on success."""
    s = settings or load_settings()
    if not s["enabled"]:
        return False
    ok = _switch("off", s["hub"])
    if ok:
        logger.info("USB hub %s power off", s["hub"])
    return ok


def hub_is_on(hub=None):
    """Return the hub's power state (True/False), or None if unreadable."""
    exe = find_uhubctl()
    if exe is None:
        return None
    hub = hub or load_settings()["hub"]
    try:
        result = subprocess.run(["sudo", "-n", exe, "-l", str(hub)],
                                capture_output=True, text=True, timeout=10)
    except (subprocess.TimeoutExpired, OSError):
        return None
    ports = [ln for ln in result.stdout.splitlines() if ln.strip().startswith("Port")]
    if not ports:
        return None
    # a port line reads e.g. "  Port 1: 02a0 power 5gbps ..." (on) or "  0080 off"
    return any(" off" not in ln for ln in ports)
