"""Measure the floor tilt of a scan's assembled cloud, and how leveling changes it.

The honest success metric from docs/imu_leveling.md: is the FLOOR horizontal?
This estimates the dominant floor plane with a RANSAC fit (robust, unlike the
crude lowest-15%-by-Z PCA) and reports the angle between its normal and +Z for:

  - the raw (device-space) cloud,
  - the cloud leveled with the OLD method (mean gravity, ``imu_up_vector``),
  - the cloud leveled with the fix (reference gravity, ``reference_gravity``).

Run (desktop dev venv, open3d required):
    ~/venv/bin/python experiments/floor_tilt.py 261011-2156
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.pointcloud import merge_2D_points, load_raw_scan  # noqa: E402
from lib.imu_orientation import (  # noqa: E402
    imu_up_vector, reference_gravity, leveling_rotation,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def floor_normal(points, frac=0.15, dist_mm=50.0, iters=2000):
    """RANSAC plane through the lowest ``frac`` of points (by Z).

    Returns (unit normal with +Z sign, inlier count, candidate count).
    """
    pts = np.asarray(points, dtype=np.float64)
    z = pts[:, 2]
    low = pts[z <= np.quantile(z, frac)][:, :3]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(low))
    model, inliers = pcd.segment_plane(distance_threshold=dist_mm,
                                       ransac_n=3, num_iterations=iters)
    n = np.asarray(model[:3], dtype=float)
    n /= np.linalg.norm(n)
    if n[2] < 0:
        n = -n
    return n, len(inliers), len(low)


def tilt_deg(normal):
    return float(np.degrees(np.arccos(np.clip(abs(normal[2]), -1, 1))))


def main():
    scan_id = sys.argv[1] if len(sys.argv) > 1 else "261011-2156"
    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    mounting = np.asarray(cfg["IMU"]["MOUNTING"], dtype=float)
    pos, rot = cfg["3D"]["CALIBRATION"]["POSITION"], cfg["3D"]["CALIBRATION"]["ROTATION"]

    path = os.path.join(ROOT, "scans", scan_id, f"{scan_id}_lidar.pkl")
    raw = load_raw_scan(path)
    print(f"scan {scan_id}: {len(raw['cartesian'])} slices from pickle")

    cloud = merge_2D_points(raw, mount=(pos, rot))
    print(f"assembled cloud: {cloud.shape[0]} points (device space, mm)")

    n_raw, inl, cand = floor_normal(cloud)
    print(f"\nRAW floor normal = {np.round(n_raw, 4).tolist()}  "
          f"tilt = {tilt_deg(n_raw):.2f} deg  ({inl}/{cand} inliers)")

    quats = raw.get("quaternions")
    if quats is None:
        print("no quaternions -> cannot level")
        return
    z_angles = raw.get("z_angles")

    for name, up in (
        ("OLD mean gravity   ", mounting @ imu_up_vector(quats)),
        ("NEW reference grav ", mounting @ reference_gravity(quats, z_angles=z_angles)),
    ):
        rot = leveling_rotation(up).as_matrix()
        leveled = cloud[:, :3] @ rot.T
        n, inl, cand = floor_normal(leveled)
        print(f"{name}: up(cloud) = {np.round(up, 4).tolist()}  "
              f"-> floor tilt = {tilt_deg(n):.2f} deg  ({inl}/{cand} inliers)")


if __name__ == "__main__":
    main()
