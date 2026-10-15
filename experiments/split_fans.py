"""Build two point clouds from the two in-plane halves of every LiDAR fan.

The LiDAR sits a little off the rotation axis, so over a full 360 deg sweep every
surface is observed from **two radial sides** (a parallax baseline of ~2x the
offset). Each fan is a full circle in its vertical plane; its beams split into the
half with in-plane radial coordinate ``x > 0`` and the half with ``x < 0``.

Assembling the two halves separately gives two clouds that **cover the same
surfaces** and should **coincide** when the LiDAR mount offset is correct. Any
systematic offset between them ("double walls") is the mount error.

    ~/venv/bin/python experiments/split_fans.py 261014-1954
    ~/venv/bin/python experiments/split_fans.py 261014-1954 --axis x --voxel 0.01

Writes ``scans/<id>/<id>_half_left.ply`` and ``..._half_right.ply``.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.pointcloud import load_raw_scan, merge_2D_points  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def split_fans(raw, axis="x"):
    """Split each 2D fan into (positive, negative) halves by an in-plane axis.

    Columns are (x, y, intensity) with ``x = -r*cos(theta)`` (in-plane radial) and
    ``y = r*sin(theta)`` (in-plane vertical). Returns two ``raw_scan``-shaped dicts.
    """
    col = {"x": 0, "y": 1}[axis]
    pos_fans, neg_fans = [], []
    for fan in raw["cartesian"]:
        fan = np.asarray(fan)
        c = fan[:, col]
        pos_fans.append(fan[c >= 0])
        neg_fans.append(fan[c < 0])
    z = raw["z_angles"]
    return ({"z_angles": z, "cartesian": pos_fans},
            {"z_angles": z, "cartesian": neg_fans})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--axis", choices=["x", "y"], default="x",
                    help="in-plane axis to split on (x = radial, default)")
    ap.add_argument("--voxel", type=float, default=0.005, help="preview voxel size (m)")
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    mount = (cfg["3D"]["CALIBRATION"]["POSITION"], cfg["3D"]["CALIBRATION"]["ROTATION"])
    scale = cfg["3D"]["SCALE"]
    scan_dir = os.path.join(ROOT, "scans", args.scan_id)
    raw = load_raw_scan(os.path.join(scan_dir, f"{args.scan_id}_lidar.pkl"))

    pos_raw, neg_raw = split_fans(raw, args.axis)
    n_pos = sum(len(f) for f in pos_raw["cartesian"])
    n_neg = sum(len(f) for f in neg_raw["cartesian"])
    print(f"{args.scan_id}: {len(raw['cartesian'])} fans; split on '{args.axis}': "
          f"{n_pos} pos / {n_neg} neg points")

    clouds = {}
    for name, sub in (("left", neg_raw), ("right", pos_raw)):
        pts = merge_2D_points(sub, mount=mount)[:, :3] * scale
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts))
        clouds[name] = pcd
        if not args.no_save:
            out = os.path.join(scan_dir, f"{args.scan_id}_half_{name}.ply")
            o3d.io.write_point_cloud(out, pcd.voxel_down_sample(args.voxel))
            print(f"  {name:5s}: {len(pcd.points)} pts -> {os.path.relpath(out, ROOT)}")

    # numeric check: do the two halves coincide? (ICP residual = mount consistency)
    a = clouds["left"].voxel_down_sample(0.02)
    b = clouds["right"].voxel_down_sample(0.02)
    reg = o3d.pipelines.registration.registration_icp(
        b, a, 0.2, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    T = reg.transformation
    Rm, t = T[:3, :3], T[:3, 3] * 1000.0
    angle = np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1)))
    w, v = np.linalg.eig(Rm)
    axis = np.real(v[:, np.argmin(np.abs(w - 1))])
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    print(f"\nleft vs right ICP: fitness={reg.fitness:.3f} rmse={reg.inlier_rmse*1000:.1f}mm")
    print(f"  rotation {angle:.2f} deg about axis=({axis[0]:+.2f},{axis[1]:+.2f},{axis[2]:+.2f})"
          f"  translation=({t[0]:+.1f},{t[1]:+.1f},{t[2]:+.1f})mm")
    print("  axis ~ Z & translation ~ 0 => halves coincide (mount ok + negligible parallax);")
    print("  a residual relative rotation is a LiDAR geometry/mount error between the halves.")

    # per-range-band translation: does the offset grow with distance (parallax / mount position)?
    print("\n  range band -> relative translation (mm):")
    dl = np.hypot(np.asarray(clouds["left"].points)[:, 0], np.asarray(clouds["left"].points)[:, 1])
    dr = np.hypot(np.asarray(clouds["right"].points)[:, 0], np.asarray(clouds["right"].points)[:, 1])
    Lp, Rp = np.asarray(clouds["left"].points), np.asarray(clouds["right"].points)
    for lo, hi in ((0, 2), (2, 4), (4, 8), (8, 16)):
        ml, mr = (dl >= lo) & (dl < hi), (dr >= lo) & (dr < hi)
        if ml.sum() < 100 or mr.sum() < 100:
            continue
        pa = o3d.geometry.PointCloud(); pa.points = o3d.utility.Vector3dVector(Lp[ml])
        pb = o3d.geometry.PointCloud(); pb.points = o3d.utility.Vector3dVector(Rp[mr])
        pa = pa.voxel_down_sample(0.02); pb = pb.voxel_down_sample(0.02)
        r = o3d.pipelines.registration.registration_icp(
            pb, pa, 0.2, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        tt = r.transformation[:3, 3] * 1000.0
        print(f"    {lo:2d}..{hi:2d}m: ({tt[0]:+6.1f},{tt[1]:+6.1f},{tt[2]:+6.1f})mm "
              f"(fitness {r.fitness:.2f})")


if __name__ == "__main__":
    main()
