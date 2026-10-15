"""Fit the LiDAR mount (3D.CALIBRATION) from the left/right fan halves.

The two in-plane halves of every fan observe the same surfaces from two radial
sides (the ~2x offset parallax), so with a correct mount the "left" and "right"
clouds **coincide**. This sweeps the mount parameters and reports the left<->right
ICP residual (rmse, relative yaw, relative shift) so the correct mount can be read
off / fit.

    ~/venv/bin/python experiments/fit_mount.py 261015-0821
    ~/venv/bin/python experiments/fit_mount.py 261015-0821 --sweep rz -1 -3 0.25
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from lib.pointcloud import load_raw_scan  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N = 500_000


def load_fans(scan_id):
    raw = load_raw_scan(os.path.join(ROOT, "scans", scan_id, f"{scan_id}_lidar.pkl"))
    # stack local (x, 0, y) points + plane index, split by in-plane x sign
    chunks, idx = [], []
    for i, fan in enumerate(raw["cartesian"]):
        pts = np.insert(np.asarray(fan, dtype=float), 1, values=0.0, axis=1)
        chunks.append(pts)
        idx.append(np.full(len(pts), i))
    P = np.vstack(chunks)
    pidx = np.concatenate(idx)
    z = np.asarray(raw["z_angles"], dtype=float)
    sel = np.random.default_rng(0).choice(len(P), min(N, len(P)), replace=False)
    return P[sel], pidx[sel], z


def assemble(P, pidx, z, pos, rot):
    rm = R.from_euler("xyz", rot, degrees=True).as_matrix()
    pl = P[:, :3] @ rm.T + np.asarray(pos)                       # mount rotation + position
    ang = -np.radians(z[pidx])                                   # revolve by -z_angle
    ca, sa = np.cos(ang), np.sin(ang)
    out = np.empty_like(pl)
    out[:, 0] = pl[:, 0] * ca - pl[:, 1] * sa
    out[:, 1] = pl[:, 0] * sa + pl[:, 1] * ca
    out[:, 2] = pl[:, 2]
    return out


def residual(P, pidx, z, pos, rot, voxel=0.02):
    pts = assemble(P, pidx, z, pos, rot) * 0.001
    pos_mask = P[:, 0] >= 0
    a = o3d.geometry.PointCloud(); a.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts[pos_mask]))
    b = o3d.geometry.PointCloud(); b.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts[~pos_mask]))
    a = a.voxel_down_sample(voxel); b = b.voxel_down_sample(voxel)
    reg = o3d.pipelines.registration.registration_icp(
        b, a, 0.2, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint())
    T = reg.transformation
    yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    return reg.fitness, reg.inlier_rmse * 1000, yaw, T[:3, 3] * 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--sweep", choices=["rx", "ry", "rz", "px", "py", "pz"], default=None)
    ap.add_argument("--lo", type=float)
    ap.add_argument("--hi", type=float)
    ap.add_argument("--step", type=float)
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    base_pos = list(cfg["3D"]["CALIBRATION"]["POSITION"])
    base_rot = list(cfg["3D"]["CALIBRATION"]["ROTATION"])
    P, pidx, z = load_fans(args.scan_id)
    print(f"{args.scan_id}: {len(P)} sampled points; mount pos={base_pos} rot={base_rot}")

    def show(pos, rot, label=""):
        fit, rmse, yaw, shift = residual(P, pidx, z, pos, rot)
        print(f"  {label:22s} rmse={rmse:6.1f}mm yaw={yaw:+.2f}deg "
              f"shift=({shift[0]:+.1f},{shift[1]:+.1f},{shift[2]:+.1f})mm")
        return rmse, yaw

    show(base_pos, base_rot, "baseline")

    if args.sweep:
        idx = {"rx": 0, "ry": 1, "rz": 2, "px": 0, "py": 1, "pz": 2}[args.sweep]
        is_rot = args.sweep[0] == "r"
        vals = np.arange(args.lo, args.hi + 1e-9, args.step)
        print(f"\nsweep {args.sweep} over {vals[0]}..{vals[-1]}:")
        for v in vals:
            pos, rot = list(base_pos), list(base_rot)
            (rot if is_rot else pos)[idx] = v
            show(pos, rot, f"{args.sweep}={v:+.2f}")


if __name__ == "__main__":
    main()
