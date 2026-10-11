"""Tests for the scan-button gesture logic (``gpio_interrupt.py``).

Only the decision logic is tested (no GPIO): a *short* press starts a scan, a
*long* press (>2 s) toggles USB power. Systemd imports the module for real, so
these also guard that importing is side-effect free.
"""
import time
from unittest.mock import MagicMock

import gpio_interrupt as btn


def test_short_press_starts_scan_and_no_usb_toggle(monkeypatch):
    scan = MagicMock()
    toggle = MagicMock()
    monkeypatch.setattr(btn, "_start_scan", scan)
    monkeypatch.setattr(btn, "_toggle_usb", toggle)

    btn.button_callback(0, btn.START_PIN, 0, 0)      # press
    time.sleep(0.1)                                  # < LONG_PRESS
    btn.button_callback(0, btn.START_PIN, 1, 0)      # release

    scan.assert_called_once()
    toggle.assert_not_called()


def test_long_press_toggles_usb_and_no_scan(monkeypatch):
    scan = MagicMock()
    toggle = MagicMock()
    monkeypatch.setattr(btn, "_start_scan", scan)
    monkeypatch.setattr(btn, "_toggle_usb", toggle)
    monkeypatch.setattr(btn, "LONG_PRESS", 0.05)

    btn.button_callback(0, btn.START_PIN, 0, 0)      # press
    time.sleep(0.2)                                  # > LONG_PRESS -> timer fires
    btn.button_callback(0, btn.START_PIN, 1, 0)      # release

    toggle.assert_called_once()
    scan.assert_not_called()


def test_toggle_usb_off_when_on(monkeypatch):
    monkeypatch.setattr(btn.usb_power, "hub_is_on", lambda hub=None: True)
    off, on = MagicMock(), MagicMock()
    monkeypatch.setattr(btn.usb_power, "hub_off", off)
    monkeypatch.setattr(btn.usb_power, "hub_on", on)

    btn._toggle_usb()
    off.assert_called_once()
    on.assert_not_called()


def test_toggle_usb_on_when_off(monkeypatch):
    monkeypatch.setattr(btn.usb_power, "hub_is_on", lambda hub=None: False)
    off, on = MagicMock(), MagicMock()
    monkeypatch.setattr(btn.usb_power, "hub_off", off)
    monkeypatch.setattr(btn.usb_power, "hub_on", on)

    btn._toggle_usb()
    on.assert_called_once()
    off.assert_not_called()
