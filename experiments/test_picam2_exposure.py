"""EV calibration & HDR bracket check (run on the Pi).

Implements the EV workflow:
  1. auto-expose, read ISO + exposure, compute the scene illumination `ev`.
  2. `ev_transform(ev)` -> (ISO, shutter) for the base frame.
  3. take a manual photo and compare its brightness to the auto one (should match).
  4. compute the bracket exposure set (e.g. -2..+2) and print it.
  5. take the bracket and report the brightness of each frame.

    ~/venv/bin/python experiments/test_picam2_exposure.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from lib.picam2_utils import Camera, bracket_exposures, ev_transform, illumination  # noqa: E402

OUT = "/tmp/picam2_exp"
os.makedirs(OUT, exist_ok=True)
STEPS = [-2, -1, 0, 1, 2]      # bracket steps in EV stops


def mean_luma(path):
    a = np.asarray(Image.open(path).convert("L"), dtype=np.float32)
    return float(a.mean())


def main():
    with Camera() as cam:
        # 1. auto-expose and read the sensor's chosen ISO + exposure
        cam.configure(still_dims=(1280, 960))
        cam._settle(frames=20, interval=0.1)

        auto = cam.camera.capture_metadata()
        auto_et = auto["ExposureTime"] / 1e9
        auto_gain = auto["AnalogueGain"] * auto["DigitalGain"]
        awb = list(auto["ColourGains"])
        ev = illumination(auto_et, auto_gain)

        auto_path = os.path.join(OUT, "auto.jpg")
        cam.take_photo(auto_path, auto_et, auto_gain, awb)
        print(f"[1+3] auto:     ET={auto_et*1000:7.2f} ms  gain={auto_gain:5.2f}  "
              f"lum={mean_luma(auto_path):5.1f}   ev={ev:.4f}")

        # 2. turn ev back into ISO + shutter
        cam._lock_3a()
        gain, exposure = ev_transform(ev)
        print(f"[2]   ev_transform -> gain={gain} ({'ISO100' if gain == 1.0 else 'ISO200'})  "
              f"exposure={exposure*1000:.2f} ms")

        # 3. manual photo should match the auto one in brightness
        ev_path = os.path.join(OUT, "ev.jpg")
        cam.take_photo(ev_path, exposure, gain, awb)
        print(f"[3]   manual:   ET={exposure*1000:7.2f} ms  gain={gain:5.2f}  "
              f"lum={mean_luma(ev_path):5.1f}")

        # 4. bracket exposure set
        bracket = bracket_exposures(ev, STEPS)
        print("[4]   bracket (step: gain, exposure):")
        for s, (g, e) in zip(STEPS, bracket):
            print(f"        {s:+d}: gain={g:4.1f}  exposure={e*1000:8.2f} ms")

        # 5. take the bracket
        print("[5]   capturing bracket ...")
        cam._lock_3a()
        for s, (g, e) in zip(STEPS, bracket):
            p = os.path.join(OUT, f"bracket_{s:+d}.jpg")
            cam._capture_at(p, e, g, awb)
            print(f"        {s:+d}: lum={mean_luma(p):5.1f}")


if __name__ == "__main__":
    main()
