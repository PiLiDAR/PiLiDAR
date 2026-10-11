"""Focused check: does manual ExposureTime / AnalogueGain actually change the
captured frame? Configures ONCE, locks 3A, then captures frames at several
exposures and reports mean brightness. If manual exposure works, brightness
should scale with exposure time (until clipping).

Run on the Pi:
  ~/venv/bin/python experiments/test_picam2_exposure.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from lib.picam2_utils import Camera  # noqa: E402

OUT = "/tmp/picam2_exp"
os.makedirs(OUT, exist_ok=True)


def mean_luma(path):
    a = np.asarray(Image.open(path).convert("L"), dtype=np.float32)
    return float(a.mean())


def main():
    with Camera() as cam:
        cam.configure(still_dims=(1280, 960))
        cam._settle(frames=20, interval=0.1)
        m = cam.camera.capture_metadata()
        base_et = m["ExposureTime"] / 1e9
        base_ag = m["AnalogueGain"]
        awb = list(m["ColourGains"])
        print("base ET=%.1fus AG=%.2f" % (base_et * 1e6, base_ag))

        cam._lock_3a()
        # let the lock take effect on one frame
        cam.camera.capture_array()

        # Find a readable exposure first (scene is dark), then bracket around it.
        for mult in (10, 50, 100, 200):
            et = base_et * mult
            p = os.path.join(OUT, "find_%d.jpg" % mult)
            cam._set_manual(et, base_ag, awb)
            cam.camera.capture_file(p)
            print("find x%-4d ET=%7.1fus  mean_luma=%.2f" % (mult, et * 1e6, mean_luma(p)))

        bright_et = base_et * 50.0
        for label, mult in [("half", 0.5), ("base", 1.0), ("double", 2.0), ("quad", 4.0)]:
            et = bright_et * mult
            p = os.path.join(OUT, "exp_%s.jpg" % label)
            cam._set_manual(et, base_ag, awb)
            cam.camera.capture_file(p)
            print("%-7s ET=%.1fus  mean_luma=%.2f" % (label, et * 1e6, mean_luma(p)))


if __name__ == "__main__":
    main()
