"""Log the IMU quaternion at every stepper angle over a full 360 deg sweep.

Writes /tmp/up_vector_sweep.csv with columns: angle_deg, qw, qx, qy, qz, norm.
The raw (un-normalized) values are kept so a corrupted / misaligned DMP stream is
visible (a healthy unit quaternion has norm ~16384).

Run on the Pi:  ~/venv/bin/python experiments/up_vector_probe.py
"""
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from lib.config import Config  # noqa: E402
from lib.imu_driver import MPU6050Wrapper  # noqa: E402
from lib.a4988_driver import A4988  # noqa: E402

OUT_CSV = "/tmp/up_vector_sweep.csv"
STEP_DEG = 5.0         # 72 samples over 360 deg
SETTLE = 0.6


def main():
    config = Config()
    config.raw_path = None

    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"),
                         config.get("IMU", "frequency"))
    stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"),
                    config.get("STEPPER", "pins", "STEP_PIN"),
                    config.get("STEPPER", "pins", "MS_PINS"),
                    delay=config.get("STEPPER", "STEP_DELAY"),
                    step_angle=config.get("STEPPER", "STEP_ANGLE"),
                    microsteps=config.get("STEPPER", "MICROSTEPS"),
                    gear_ratio=config.get("STEPPER", "GEAR_RATIO"))

    rows = []
    try:
        time.sleep(0.5)
        for _ in range(int(360 / STEP_DEG) + 1):
            angle = round(stepper.get_current_angle(), 2)
            q = imu.get_quat_values().astype(float)          # raw int16 wxyz
            norm = float(np.linalg.norm(q))
            rows.append((angle, q[0], q[1], q[2], q[3], norm))
            stepper.move_angle(STEP_DEG)
            time.sleep(SETTLE)
    finally:
        stepper.close()
        imu.close()

    with open(OUT_CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["angle_deg", "qw", "qx", "qy", "qz", "norm"])
        w.writerows(rows)

    norms = np.array([r[5] for r in rows])
    print(f"wrote {len(rows)} rows -> {OUT_CSV}")
    print(f"norm: mean={norms.mean():.0f}  min={norms.min():.0f}  max={norms.max():.0f}  "
          f"(healthy ~16384)")


if __name__ == "__main__":
    main()
