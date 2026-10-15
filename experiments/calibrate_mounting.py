"""Solve and validate the IMU -> cloud MOUNTING rotation from a scan.

The mounting is calibration data (``config.json`` -> ``IMU.MOUNTING``). It is a
*proper* rotation that maps a vector from the IMU body frame into the assembled
cloud frame, and it is constrained by two physical facts:

  1. the turntable axis (in the body frame) maps to the cloud **+Z** (the cloud is
     assembled in device space, so its up-axis is +Z by construction);
  2. the gravity direction at the reference attitude maps to the **floor normal**
     measured from the assembled cloud.

.. warning::
   Calibrate with a **clearly tilted** rig (~10-15 deg). On a level rig the axis
   and gravity are nearly parallel and the two constraints become degenerate.

Procedure per scan:
  - load the raw scan (needs ``quaternions`` + ``z_angles``),
  - fit the turntable axis in the body frame (``fit_turntable_axis``),
  - take gravity at the reference attitude (``reference_gravity``),
  - assemble the cloud and measure its floor normal (RANSAC),
  - solve the mounting (``solve_mounting``) and report the residuals.

To *calibrate*: run on a tilted scan, take the printed matrix into config.json,
then run again on other scans (ideally one level) and confirm the residual floor
tilt after leveling stays small.

Run:
    ~/venv/bin/python experiments/calibrate_mounting.py 261011-2031
    ~/venv/bin/python experiments/calibrate_mounting.py 261011-2031 --use-config
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.imu_orientation import (  # noqa: E402
    fit_turntable_axis, imu_up_vector, leveling_rotation, reference_gravity, solve_mounting,
)
from lib.pointcloud import load_raw_scan, merge_2D_points  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
Z = np.array([0.0, 0.0, 1.0])


def _angle(u, v):
    u = np.asarray(u, float) / np.linalg.norm(u)
    v = np.asarray(v, float) / np.linalg.norm(v)
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(u, v)), -1, 1))))


def floor_normal(points, frac=0.15, dist_mm=50.0, iters=8000):
    """RANSAC plane through the lowest ``frac`` of points (by Z); +Z-signed normal."""
    p = np.asarray(points, float)[:, :3]
    low = p[p[:, 2] <= np.quantile(p[:, 2], frac)]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(low))
    model, inliers = pcd.segment_plane(distance_threshold=dist_mm, ransac_n=3,
                                       num_iterations=iters)
    n = np.asarray(model[:3], float)
    n /= np.linalg.norm(n)
    return (-n if n[2] < 0 else n), len(inliers), len(low)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--use-config", action="store_true",
                    help="evaluate the matrix already in config.json instead of solving")
    ap.add_argument("--axis", choices=["accel", "incremental"], default="accel",
                    help="which axis fit to use (accel = gravity cone, drift-free)")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    raw = load_raw_scan(os.path.join(ROOT, "scans", args.scan_id, f"{args.scan_id}_lidar.pkl"))
    if "quaternions" not in raw:
        sys.exit(f"scan {args.scan_id} has no quaternions — cannot calibrate")
    quats = np.asarray(raw["quaternions"], float)
    z_angles = np.asarray(raw["z_angles"], float)

    cloud = merge_2D_points(raw, position_offset=(0, cfg["3D"]["Y_OFFSET"], 0),
                            angle_offset=cfg["3D"]["LIDAR_OFFSET_ANGLE"])
    n_floor, inl, cand = floor_normal(cloud)

    fit = fit_turntable_axis(quats)
    g_ref = reference_gravity(quats, z_angles=z_angles)
    axis = (fit.c_body_accel if args.axis == "accel" else fit.c_body).copy()
    if np.dot(axis, g_ref) < 0:
        axis = -axis

    print(f"\nscan {args.scan_id}: {len(quats)} quats, {cloud.shape[0]} cloud points")
    print(f"  fit: tilt={fit.tilt_deg:.2f} deg  spread={fit.spread_deg:.2f} deg  "
          f"step={fit.step_deg:.2f} deg")
    print(f"  axis ({args.axis:11s}) = {np.round(axis, 4).tolist()}")
    print(f"  g_ref (reference)  = {np.round(g_ref, 4).tolist()}")
    print(f"  angle(axis, g_ref) = {_angle(axis, g_ref):.2f} deg "
          f"(near 0 => degenerate calibration; tilt the rig)")
    print(f"  floor normal       = {np.round(n_floor, 4).tolist()}  "
          f"tilt={_angle(n_floor, Z):.2f} deg  ({inl}/{cand} inliers)")

    if args.use_config:
        M = np.asarray(cfg["IMU"]["MOUNTING"], float)
        print("\n[--use-config] evaluating config IMU.MOUNTING:")
    else:
        M = solve_mounting(axis, g_ref, gravity_cloud=n_floor)
        print("\nSOLVED MOUNTING (paste into config.json -> IMU.MOUNTING):")
        for row in M:
            print("        [" + ", ".join(f"{v: .4f}" for v in row) + "],")

    up = M @ g_ref
    rot = leveling_rotation(up).as_matrix()
    n_after, _, _ = floor_normal(cloud[:, :3] @ rot.T)
    print(f"  M @ axis  = {np.round(M @ axis, 4).tolist()}  (want [0,0,1])")
    print(f"  M @ g_ref = {np.round(up, 4).tolist()}  err vs floor = {_angle(up, n_floor):.2f} deg")
    print(f"  floor tilt AFTER leveling = {_angle(n_after, Z):.2f} deg")


if __name__ == "__main__":
    main()