"""On-Pi smoke test for lib/picam2_utils.py.

Exercises the full Camera flow against the real IMX477:
  1. configure + let 3A settle
  2. read converged ExposureTime / AnalogueGain / DigitalGain / ColourGains
  3. lock 3A
  4. one manual still
  5. a small HDR bracket (AEB=1, AEB_stops=1)

Run on the Pi:  python3 experiments/test_picam2_pi.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib.picam2_utils import Camera  # noqa: E402

OUT = "/tmp/picam2_test"
os.makedirs(OUT, exist_ok=True)


def main():
    preview_dims = (640, 480)   # small + fast for a smoke test
    still_dims = (1280, 960)

    with Camera() as cam:
        # 1-3. settle, read, lock (at preview resolution)
        cam.configure(still_dims=preview_dims)
        cam._settle(frames=15, interval=0.1)
        meta = cam.camera.capture_metadata()
        print("== converged 3A ==")
        print("  ExposureTime : %d ns (%.4f s)" % (meta["ExposureTime"], meta["ExposureTime"] / 1e9))
        print("  AnalogueGain : %.3f" % meta["AnalogueGain"])
        print("  DigitalGain  : %.3f" % meta["DigitalGain"])
        print("  ColourGains  : [%.3f, %.3f]" % (meta["ColourGains"][0], meta["ColourGains"][1]))

        exposure_time = meta["ExposureTime"] / 1e9
        gain = meta["AnalogueGain"] * meta["DigitalGain"]
        awbgains = list(meta["ColourGains"])
        cam._lock_3a()
        print("  locked 3A; total gain = %.3f" % gain)

        # 4. one manual still at full resolution
        cam.configure(still_dims=still_dims)
        p1 = os.path.join(OUT, "manual.jpg")
        cam.take_photo(p1, exposure_time, gain, awbgains)
        print("  wrote", p1, os.path.getsize(p1), "bytes")

        # 5. small HDR bracket (base path must carry the .jpg extension)
        base = os.path.join(OUT, "hdr.jpg")
        paths = cam.take_hdr(base, AEB=1, AEB_stops=1,
                             exposure_time=exposure_time, gain=gain,
                             awbgains=awbgains)
        for p in paths:
            print("  wrote", p, os.path.getsize(p), "bytes")

    print("\nDONE")


if __name__ == "__main__":
    t0 = time.time()
    main()
    print("elapsed %.1f s" % (time.time() - t0))
