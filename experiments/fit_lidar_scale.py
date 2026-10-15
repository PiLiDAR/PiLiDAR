"""Fit the LiDAR's beam-angle scale / range scale from the left/right fan halves.

The two in-plane halves of every fan see the same surfaces from two radial sides,
so with correct LiDAR calibration they coincide. The measured left<->right
residual GROWS with distance (5 mm at 1 m -> 17 mm at 8 m), which is the signature
of a **distance-proportional** error: either the beam angle is scaled (angular) or
the range is scaled. This sweeps both and reports the residual-vs-distance slope
(and the overall rmse) so the correct correction can be read off.

    ~/venv/bin/python experiments/fit_lidar_scale.py 261015-1213
    ~/venv/bin/python experiments/fit_lidar_scale.py 261015-1213 --sweep angle --lo 0.98 --hi 1.02 --step 0.005
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.pointcloud import load_raw_scan  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
N = 400_000


def load(scan_id):
    raw = load_raw_scan(os.path.join(ROOT, "scans", scan_id, f"{scan_id}_lidar.pkl"))
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


def apply_correction(P, angle_scale=1.0, range_scale=1.0):
    """Scale the beam angle and/or the range of each 2D fan point.

    The fan columns are (x, y) = (-r cos(theta), r sin(theta)). Scaling the beam
    angle means theta -> scale*theta; scaling the range means r -> scale*r.
    """
    p = P.copy()
    x, y = p[:, 0], p[:, 1]
    r = np.hypot(x, y)
    theta = np.arctan2(y, -x)                       # recover the beam angle
    r2 = r * range_scale
    th2 = theta * angle_scale
    p[:, 0] = -r2 * np.cos(th2)
    p[:, 1] = r2 * np.sin(th2)
    return p


def assemble(P, pidx, z, mount_pos, mount_rot):
    from scipy.spatial.transform import Rotation as R
    rm = R.from_euler("xyz", mount_rot, degrees=True).as_matrix()
    pl = P[:, :3] @ rm.T + np.asarray(mount_pos)
    ang = -np.radians(z[pidx])
    ca, sa = np.cos(ang), np.sin(ang)
    out = np.empty_like(pl)
    out[:, 0] = pl[:, 0] * ca - pl[:, 1] * sa
    out[:, 1] = pl[:, 0] * sa + pl[:, 1] * ca
    out[:, 2] = pl[:, 2]
    return out


def residual_profile(P, pidx, z, pos, rot, angle_scale, range_scale, voxel=0.02):
    p = apply_correction(P, angle_scale, range_scale)
    pts = assemble(p, pidx, z, pos, rot) * 0.001
    m_pos = P[:, 0] >= 0
    a = o3d.geometry.PointCloud(); a.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts[m_pos]))
    b = o3d.geometry.PointCloud(); b.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts[~m_pos]))
    a = a.voxel_down_sample(voxel); b = b.voxel_down_sample(voxel)
    reg = o3d.pipelines.registration.registration_icp(
        b, a, 0.2, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint())
    b.transform(reg.transformation)
    A = np.asarray(a.points); B = np.asarray(b.points)
    tree = o3d.geometry.KDTreeFlann(a)
    rad = np.linalg.norm(B[:, :2], axis=1)
    d = np.sqrt([tree.search_knn_vector_3d(q, 1)[2][0] for q in B]) * 1000  # mm
    # only inlier matches (d < 100 mm): occluded regions have no counterpart and
    # would otherwise dominate the slope
    m = (rad > 1) & (rad < 8) & (d < 100)
    slope = np.polyfit(rad[m], d[m], 1)[0] if m.sum() > 100 else float("nan")
    return reg.inlier_rmse * 1000, float(np.median(d)), slope


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--sweep", choices=["angle", "range"], default=None)
    ap.add_argument("--lo", type=float, default=0.98)
    ap.add_argument("--hi", type=float, default=1.02)
    ap.add_argument("--step", type=float, default=0.005)
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    pos = cfg["3D"]["CALIBRATION"]["POSITION"]
    rot = cfg["3D"]["CALIBRATION"]["ROTATION"]
    P, pidx, z = load(args.scan_id)
    print(f"{args.scan_id}: {len(P)} pts   rmse / median / slope(mm per m)")

    def show(a, r, label=""):
        rmse, med, slope = residual_profile(P, pidx, z, pos, rot, a, r)
        print(f"  {label:22s} rmse={rmse:6.1f}  median={med:5.1f}  slope={slope:+7.1f}")
        return rmse, slope

    show(1.0, 1.0, "baseline")

    if args.sweep:
        vals = np.arange(args.lo, args.hi + 1e-9, args.step)
        best = None
        for v in vals:
            a, r = (v, 1.0) if args.sweep == "angle" else (1.0, v)
            rmse, slope = show(a, r, f"{args.sweep}={v:.4f}")
            score = abs(slope)
            if best is None or score < best[0]:
                best = (score, v)
        print(f"\nflattest slope at {args.sweep} = {best[1]:.4f}")


if __name__ == "__main__":
    main()
