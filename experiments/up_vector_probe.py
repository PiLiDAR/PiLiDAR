"""Log one IMU quaternion per LiDAR plane during a real sweep.

Mirrors the scan loop (read_loop at every plane boundary) but instead of moving
the stepper it just records the IMU quaternion, so we can fit the turntable axis
from a full sweep and cross-check the incremental-rotation and gravity methods.

Run on the Pi:  ~/venv/bin/python experiments/up_vector_probe.py
Writes:         /tmp/up_vector_probe.npz  (quaternions, z_angles)
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from lib.config import Config  # noqa: E402
from lib.imu_driver import MPU6050Wrapper  # noqa: E402
from lib.lidar_driver import Lidar  # noqa: E402
from lib import usb_power  # noqa: E402
from lib.imu_orientation import fit_turntable_axis  # noqa: E402

OUT = "/tmp/up_vector_probe.npz"
MAX_PLANES = 200      # stop early; we don't move the stepper

STOP = False


def main():
    if not usb_power.lidar_on():
        print("could not power the LiDAR")
        return

    config = Config()
    config.init(scan_id="_probe")

    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"),
                         config.get("IMU", "frequency"))
    time.sleep(0.5)
    lidar = Lidar(config, visualization=None)
    lidar.step_at_angle = None            # plain full revolutions

    quats, angles = [], []
    lidar.start_reader()
    try:
        planes = 0
        while planes < MAX_PLANES:
            if lidar.out_i >= lidar.out_len:
                quats.append(imu.get_quat_values())
                angles.append(round(float(np.degrees(lidar.angle_package[0])), 2))
                lidar.out_i = 0
                planes += 1
            if lidar.read_from_queue(timeout=1.0):
                lidar.out_i += 1
    finally:
        lidar.stop_reader()
        lidar.serial_connection.close()
        imu.close()
        usb_power.lidar_off()

    quats = np.array(quats, dtype=float)
    np.savez(OUT, quaternions=quats, z_angles=np.array(angles))
    print(f"logged {len(quats)} quaternion samples -> {OUT}")

    fit = fit_turntable_axis(quats)
    print("turntable axis fit:")
    print(f"  c_body        = [{fit.c_body[0]:+.4f} {fit.c_body[1]:+.4f} {fit.c_body[2]:+.4f}]")
    print(f"  c_body_accel  = [{fit.c_body_accel[0]:+.4f} {fit.c_body_accel[1]:+.4f} {fit.c_body_accel[2]:+.4f}]")
    print(f"  w_world       = [{fit.w_world[0]:+.4f} {fit.w_world[1]:+.4f} {fit.w_world[2]:+.4f}]")
    print(f"  |c_body.c_accel| = {abs(np.dot(fit.c_body, fit.c_body_accel)):.4f}  (1.0 = agree)")
    print(f"  tilt_deg = {fit.tilt_deg:.2f}  spread_deg = {fit.spread_deg:.3f}  step_deg = {fit.step_deg:.4f}  n = {fit.n}")
    print(f"  z_angles span: {angles[0]:.1f}..{angles[-1]:.1f} deg")


if __name__ == "__main__":
    main()
