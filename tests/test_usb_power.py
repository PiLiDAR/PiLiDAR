"""Tests for the LiDAR USB power control (``lib/usb_power.py``).

These are desktop/CI tests of the config plumbing only — they never invoke
uhubctl or touch hardware.
"""
import json

from lib import usb_power


def test_load_settings_reads_project_config():
    s = usb_power.load_settings()
    assert set(s) == {"enabled", "hub", "port", "delay", "device"}
    assert s["hub"] == "1-1"
    assert s["port"] == 1
    # device comes from the configured LiDAR device port
    assert s["device"] == "/dev/ttyUSB0"


def test_load_settings_defaults_when_block_missing(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"LIDAR": {"DEVICE": "STL27L", "STL27L": {"PORT": "/dev/ttyUSB9"}}}))

    s = usb_power.load_settings(str(cfg))
    assert s["enabled"] is False      # no USB_POWER block -> disabled
    assert s["hub"] == "1-1"          # sane defaults
    assert s["port"] == 1
    assert s["delay"] == 5
    assert s["device"] == "/dev/ttyUSB9"


def test_find_uhubctl_returns_path_or_none():
    path = usb_power.find_uhubctl()
    assert path is None or isinstance(path, str)


def test_lidar_on_noop_when_disabled(monkeypatch):
    """A disabled USB_POWER block must not shell out to uhubctl."""
    called = {"n": 0}
    monkeypatch.setattr(usb_power, "_switch", lambda *a, **k: called.__setitem__("n", called["n"] + 1))

    s = {"enabled": False, "hub": "1-1", "port": 1, "delay": 0, "device": "/dev/null"}
    assert usb_power.lidar_on(s) is False
    assert usb_power.lidar_off(s) is False
    assert called["n"] == 0
