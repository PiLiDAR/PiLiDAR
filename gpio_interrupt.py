'''
GPIO button interrupt.

The scan button has two actions, both driven by lgpio edge callbacks (no
polling loop, ~0% idle CPU):

* **short press**       -> start a scan (``PiLiDAR.py``, one at a time)
* **long press (>2 s)** -> toggle USB-A hub power, so a USB stick can be
  attached for dumping scans. The STL27L LiDAR is USB-powered and the Pi 4 can
  only switch the *whole* USB-A hub, so keeping the LiDAR off while idle means
  USB-A is unpowered — the long press is the deliberate "give me USB-A" gesture
  (see ``lib/usb_power.py`` and ``docs/protocol.md``).
'''

import os
import subprocess
import sys
import threading
import time

# make ``lib`` importable when run by systemd (working dir may not be the repo)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import usb_power

START_PIN   = 17
VENV_PYTHON = "/home/pi/venv/bin/python"
MAIN_SCRIPT = "/home/pi/PiLiDAR/PiLiDAR.py"

# a press held longer than this toggles USB power instead of starting a scan
LONG_PRESS  = 2.0        # seconds
DEBOUNCE_US = 200_000    # 200 ms hardware debounce for the mechanical switch

process = None
_state = {"timer": None, "long_fired": False}
_lock = threading.Lock()


def _start_scan():
    global process

    # Check if there is an existing process
    if process is not None:
        return_code = process.poll()
        # If the return code is None, the process is still running
        if return_code is None:
            print("Process is still running, skipping button press")
            return
        else:
            print("Process has finished, return code:", return_code)
            # the scan process powers the LiDAR off on exit too; this covers the
            # case where it died before reaching its own cleanup
            usb_power.lidar_off()

    # Start a new process with the main script at higher priority
    nice_value = -10
    process = subprocess.Popen(["nice", "-n", str(nice_value), VENV_PYTHON, MAIN_SCRIPT])
    print("Started a new process, pid:", process.pid)


def _toggle_usb():
    state = usb_power.hub_is_on()
    if state:                       # on -> off
        usb_power.hub_off()
        print("USB power OFF")
    else:                           # off (or unknown) -> on
        usb_power.hub_on()
        print("USB power ON (attach a USB stick to dump scans)")


def _on_long_press():
    with _lock:
        _state["long_fired"] = True
        _state["timer"] = None
    print(f"Long press (> {LONG_PRESS:g} s)")
    _toggle_usb()


def button_callback(chip, gpio, level, tick):
    """lgpio callback thread: level 0 = pressed, 1 = released."""
    if level == 0:                  # falling edge -> pressed; arm long-press timer
        with _lock:
            _state["long_fired"] = False
            _state["timer"] = threading.Timer(LONG_PRESS, _on_long_press)
            _state["timer"].start()
    else:                           # rising edge -> released
        with _lock:
            timer = _state["timer"]
            _state["timer"] = None
            long_fired = _state["long_fired"]
        if timer is not None:
            timer.cancel()
        if not long_fired:          # released early -> short press = scan
            _start_scan()


# Open gpiochip 0 and claim the button pin for both edges with an internal
# pull-up (the button grounds the pin on press).
def main():
    import lgpio  # lazy: keeps this module importable on desktop (no lgpio)

    # Cut the LiDAR USB power at boot: the STL27L is powered from USB and must
    # not spin (or draw power) while idle. PiLiDAR.py powers it back on for the
    # acquisition phase and off again once the scan data is saved.
    usb_power.hub_off()

    handle = lgpio.gpiochip_open(0)
    if handle < 0:
        raise SystemExit(f"could not open gpiochip 0 ({lgpio.error_text(handle)})")

    err = lgpio.gpio_claim_alert(handle, START_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
    if err < 0:
        raise SystemExit(f"could not claim GPIO {START_PIN} ({lgpio.error_text(err)})")

    lgpio.gpio_set_debounce_micros(handle, START_PIN, DEBOUNCE_US)

    cb = lgpio.callback(handle, START_PIN, lgpio.BOTH_EDGES, button_callback)

    # Block in the main thread — the callback does the work, ~0% CPU while idle.
    try:
        print(f"Waiting for button press... (short: scan, long >{LONG_PRESS:g}s: toggle USB)")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        cb.cancel()
        lgpio.gpio_free(handle, START_PIN)
        lgpio.gpiochip_close(handle)


if __name__ == "__main__":
    main()
