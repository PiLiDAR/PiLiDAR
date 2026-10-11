"""Verify the HDR bracket actually applies different exposure/gain per frame.

Confirms shutter-for-dark (negative EV -> longer ExposureTime) and
gain-for-bright (positive EV -> higher AnalogueGain), and that libcamera
honours the requested controls.

Run on the Pi:  python3 experiments/test_picam2_bracket.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib.picam2_utils import Camera  # noqa: E402


def main():
    with Camera() as cam:
        cam.configure(still_dims=(640, 480))
        cam._settle(frames=15, interval=0.1)
        m = cam.camera.capture_metadata()
        et = m["ExposureTime"] / 1e9
        g = m["AnalogueGain"] * m["DigitalGain"]
        awb = list(m["ColourGains"])
        cam._lock_3a()

        sm = 2 ** 1  # AEB_stops=1
        for step in (-1, 0, 1):
            if step < 0:
                het, hg = et * (sm ** step), g
            else:
                het, hg = et, g * (sm ** step)
            cam._set_manual(het, hg, awb)
            cam.camera.capture_array()  # force a frame with the new controls
            md = cam.camera.capture_metadata()
            print("step %+d: requested ET=%.1fus gain=%.2f | applied ET=%.1fus AnalogueGain=%.2f"
                  % (step, het * 1e6, hg, md["ExposureTime"] / 1e6, md["AnalogueGain"]))


if __name__ == "__main__":
    main()
