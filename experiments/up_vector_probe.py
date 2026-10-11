"""Validate: is the turntable's up-axis recoverable from IMU quaternions?

Rotates the stepper in steps and samples the MPU6050 after each, then computes
the incremental-rotation axis between consecutive samples. If the IMU is on the
rotating turntable, every incremental axis should be the same axis (the
turntable's up), regardless of orientation/mounting.

Run on the Pi:  ~/venv/bin/python experiments/up_vector_probe.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from lib.config import Config  # noqa: E402
from lib.imu_driver import MPU6050Wrapper  # noqa: E402
from lib.a4988_driver import A4988  # noqa: E402

N = 12          # samples
STEP_DEG = 10   # degrees per move


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

    quats = []
    try:
        time.sleep(0.5)
        for i in range(N):
            quats.append(imu.get_quat_values().astype(float))
            stepper.move_angle(STEP_DEG)
            time.sleep(1.0)
    finally:
        stepper.close()
        imu.close()

    quats = np.array(quats)
    # driver returns raw int16 DMP quaternions; scale to unit (and xyzw for scipy)
    quats = quats / 16384.0
    print("quat magnitudes:", np.round(np.linalg.norm(quats, axis=1), 3))
    rots = R.from_quat(quats[:, [1, 2, 3, 0]])

    print(f"{len(quats)} samples, {STEP_DEG} deg/step")
    print("incremental rotation axis (should be constant = up):")
    axes = []
    for i in range(len(rots) - 1):
        rel = rots[i].inv() * rots[i + 1]
        rotvec = rel.as_rotvec()
        ang = np.degrees(np.linalg.norm(rotvec))
        axis = rotvec / np.linalg.norm(rotvec)
        axes.append(axis)
        print(f"  {i:2d}->{i+1:2d}: angle={ang:6.2f} deg  axis=[{axis[0]:+.3f} {axis[1]:+.3f} {axis[2]:+.3f}]")

    axes = np.array(axes)
    mean_axis = axes.mean(0)
    mean_axis /= np.linalg.norm(mean_axis)
    # sign-align before averaging consistency check
    aligned = np.array([a if np.dot(a, mean_axis) > 0 else -a for a in axes])
    mean_axis = aligned.mean(0)
    mean_axis /= np.linalg.norm(mean_axis)
    spread = np.degrees(np.arccos(np.clip(aligned @ mean_axis, -1, 1)))
    print(f"\nmean up axis (IMU frame): [{mean_axis[0]:+.4f} {mean_axis[1]:+.4f} {mean_axis[2]:+.4f}]")
    print(f"axis spread (deg): max={spread.max():.2f}  mean={spread.mean():.2f}  std={spread.std():.2f}")
    print(f"per-step rotation angle: mean={np.degrees([np.linalg.norm(r.as_rotvec()) for r in (rots[i].inv()*rots[i+1] for i in range(len(rots)-1))]).mean():.2f} deg")


if __name__ == "__main__":
    main()
