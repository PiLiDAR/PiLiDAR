'''
GPIO button interrupt: starts a scan when the button is pressed.

Uses the lgpio C-API (``pip install lgpio``) instead of the deprecated
RPi.GPIO. The button edge is handled by an lgpio callback thread, so the
main thread just sleeps — no polling loop — and idle CPU usage is ~0%.
'''

import os
import subprocess
import sys
import time

import lgpio

# make ``lib`` importable when run by systemd (working dir may not be the repo)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lib import usb_power

START_PIN   = 17
VENV_PYTHON = "/home/pi/venv/bin/python"
MAIN_SCRIPT = "/home/pi/PiLiDAR/PiLiDAR.py"


process = None

# Cut the LiDAR USB power at boot: the STL27L is powered from USB and must not
# spin (or draw power) while idle. PiLiDAR.py powers it back on for the
# acquisition phase and off again once the scan data is saved.
usb_power.lidar_off()


def start_callback(chip, gpio, level, tick):
    """Runs in the lgpio callback thread on a falling edge (button press)."""
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


# Open gpiochip 0 and claim the button pin for falling-edge alerts with an
# internal pull-up (the button grounds the pin on press).
handle = lgpio.gpiochip_open(0)
if handle < 0:
    raise SystemExit(f"could not open gpiochip 0 ({lgpio.error_text(handle)})")

err = lgpio.gpio_claim_alert(handle, START_PIN, lgpio.FALLING_EDGE, lgpio.SET_PULL_UP)
if err < 0:
    raise SystemExit(f"could not claim GPIO {START_PIN} ({lgpio.error_text(err)})")

# 200 ms debounce for the mechanical switch (microseconds).
lgpio.gpio_set_debounce_micros(handle, START_PIN, 200_000)

cb = lgpio.callback(handle, START_PIN, lgpio.FALLING_EDGE, start_callback)

# Block in the main thread — the callback does the work, ~0% CPU while idle.
try:
    print("Waiting for button press...")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    cb.cancel()
    lgpio.gpio_free(handle, START_PIN)
    lgpio.gpiochip_close(handle)
