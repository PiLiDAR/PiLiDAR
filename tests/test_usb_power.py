"""Tests for the LiDAR USB power control (``lib/usb_power.py``).

These are desktop/CI tests of the config plumbing only — they never invoke
uhubctl or touch hardware.
"""
import json

from lib import usb_power


def test_load_settings_reads_project_config():
    s = usb_power.load_settings()
    assert set(s) == {"enabled", "hub", "delay", "device"}
    assert s["hub"] == "2"            # Pi 4: whole-hub switching on the VL805
    # device prefers the stable by-id path when it exists
    assert s["device"].startswith("/dev/serial/by-id/") or s["device"] == "/dev/ttyUSB0"


def test_load_settings_defaults_when_block_missing(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"LIDAR": {"DEVICE": "STL27L", "STL27L": {"PORT": "/dev/ttyUSB9"}}}))

    s = usb_power.load_settings(str(cfg))
    assert s["enabled"] is False      # no USB_POWER block -> disabled
    assert s["hub"] == "2"            # sane defaults
    assert s["delay"] == 5
    assert s["device"] == "/dev/ttyUSB9"  # no PORT_BY_ID -> raw port


def test_resolve_serial_device_prefers_by_id(monkeypatch):
    """PORT_BY_ID wins when a matching node exists, else the raw PORT is used."""
    # no glob -> raw port
    assert usb_power.resolve_serial_device("/dev/ttyUSB0", None) == "/dev/ttyUSB0"
    # glob matches nothing -> raw port
    assert usb_power.resolve_serial_device("/dev/ttyUSB0", "no-such-device-*") == "/dev/ttyUSB0"
    # glob matches -> the stable by-id path
    match = "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_abc-if00-port0"
    monkeypatch.setattr(usb_power.glob, "glob", lambda pattern: [match])
    assert usb_power.resolve_serial_device("/dev/ttyUSB0", "usb-Silicon_Labs_*") == match


def test_find_uhubctl_returns_path_or_none():
    path = usb_power.find_uhubctl()
    assert path is None or isinstance(path, str)


def test_lidar_on_noop_when_disabled(monkeypatch):
    """A disabled USB_POWER block must not shell out to uhubctl."""
    called = {"n": 0}
    monkeypatch.setattr(usb_power, "_switch", lambda *a, **k: called.__setitem__("n", called["n"] + 1))

    s = {"enabled": False, "hub": "2", "delay": 0, "device": "/dev/null"}
    assert usb_power.lidar_on(s) is False
    assert usb_power.lidar_off(s) is False
    assert called["n"] == 0
