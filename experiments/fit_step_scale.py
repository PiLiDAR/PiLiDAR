"""Diagnose the per-step rotation of a 360 deg sweep.

Hypothesis: the assembled cloud "fans out" because each plane is revolved by the
*reported* stepper angle, which may not match the *actual* rotation (gear ratio
error, dropped steps, wrong nominal step). For a 360 deg sweep the first and second
halves observe the same surfaces, so if we rotate the second half by 180 deg it
should land on the first half. A residual yaw means the rotation scale is off:

    angle_used = SCALE * z + OFFSET

This sweeps SCALE (and optionally OFFSET) and reports the ICP residual of the
180-rotated second half against the first half, plus the "double-wall" spread.

Usage:
    ~/venv/bin/python experiments/fit_step_scale.py 261014-1954
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.pointcloud import load_raw_scan  # noqa: E402
from experiments.tune_angle_step import assemble_local, revolve  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N_SAMPLE = 400_000


def load(scan_id, cfg, sample=N_SAMPLE):
    raw = load_raw_scan(os.path.join(ROOT, "scans", scan_id, f"{scan_id}_lidar.pkl"))
    P, pidx = assemble_local(raw, cfg)
    if len(P) > sample:
        sel = np.random.default_rng(0).choice(len(P), sample, replace=False)
        P, pidx = P[sel], pidx[sel]
    return P, pidx, np.asarray(raw["z_angles"], dtype=float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    P, pidx, z = load(args.scan_id, cfg)
    mid = int(pidx.max() // 2)
    print(f"{args.scan_id}: {len(z)} planes, midpoint plane {mid} (z={z[mid]:.1f} deg)")

    def build(scale, offset):
        pts = revolve(P, pidx, z, scale, offset)[:, :3] * 0.001
        return pts

    def residual(scale, offset=0.0):
        pts = build(scale, offset)
        m1 = pidx <= mid
        m2 = ~m1
        a = o3d.geometry.PointCloud()
        a.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts[m1]))
        b = o3d.geometry.PointCloud()
        b.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts[m2]))
        a = a.voxel_down_sample(0.02)
        b = b.voxel_down_sample(0.02)
        Rz = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        b.rotate(Rz[:3, :3])
        reg = o3d.pipelines.registration.registration_icp(
            b, a, 0.15, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        T = reg.transformation
        yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
        return reg.fitness, reg.inlier_rmse, yaw, T

    print("\nSCALE sweep (fitness up = better, yaw ~ 0 = correct rotation):")
    best = None
    for s in np.arange(0.990, 1.0101, 0.001):
        fit, rmse, yaw, T = residual(s)
        tag = ""
        if best is None or fit > best[0]:
            best = (fit, s, yaw, rmse)
            tag = "  <-- best"
        print(f"  SCALE={s:.4f}  fitness={fit:.3f}  rmse={rmse*1000:6.1f}mm  resid_yaw={yaw:+.3f} deg{tag}")
    print(f"\nBEST SCALE = {best[1]:.4f}  (fitness {best[0]:.3f}, rmse {best[3]*1000:.1f}mm, yaw {best[2]:+.3f} deg)")


if __name__ == "__main__":
    main()
