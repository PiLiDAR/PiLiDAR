"""Check a >360 deg sweep by splitting it into overlapping azimuth arcs.

For a sweep of more than 360 deg the FIRST arc (azimuth 0..360) and the SECOND arc
(the overrun, 360..SCAN_ANGLE) re-observe the same surfaces from the same side. If
the per-step angle is right, the two clouds land **exactly on top of each other**
in the overlap; a wrong step angle makes them **fan apart proportionally to the
azimuth difference** (the classic symptom).

This splits the assembled cloud by azimuth = ``z_angle`` into:
  * ``arc1`` — planes with z_angle in [0, 360)
  * ``arc2`` — planes with z_angle in [360, SCAN_ANGLE)   (the overrun)

and reports their coincidence in the overlap (arc2 shifted back by 360 deg).

    ~/venv/bin/python experiments/split_overlap.py 261015-XXXX
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


def split_arcs(raw):
    """Split the sweep into arc1 (z in [0,360)) and arc2 (z in [360, ...))."""
    z = np.asarray(raw["z_angles"], dtype=float)
    fans = raw["cartesian"]
    arc1 = {"z_angles": [], "cartesian": []}
    arc2 = {"z_angles": [], "cartesian": []}
    for zi, fan in zip(z, fans):
        (arc1 if zi < 360.0 else arc2)["z_angles"].append(zi)
        (arc1 if zi < 360.0 else arc2)["cartesian"].append(fan)
    return arc1, arc2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--voxel", type=float, default=0.005)
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    mount = (cfg["3D"]["CALIBRATION"]["POSITION"], cfg["3D"]["CALIBRATION"]["ROTATION"])
    scale = cfg["3D"]["SCALE"]
    scan_dir = os.path.join(ROOT, "scans", args.scan_id)
    raw = load_raw_scan(os.path.join(scan_dir, f"{args.scan_id}_lidar.pkl"))

    z = np.asarray(raw["z_angles"], dtype=float)
    print(f"{args.scan_id}: {len(z)} planes, z_angle {z.min():.2f} .. {z.max():.2f} deg "
          f"(overrun {max(0, z.max()-360):.2f} deg)")

    arc1, arc2 = split_arcs(raw)
    print(f"  arc1 (0..360): {len(arc1['z_angles'])} planes")
    print(f"  arc2 (360..): {len(arc2['z_angles'])} planes")
    if not arc2["z_angles"]:
        print("  no overrun -> cannot check the overlap (need SCAN_ANGLE > 360)")
        return

    p1 = merge_2D_points(arc1, mount=mount)[:, :3] * scale
    p2 = merge_2D_points(arc2, mount=mount)[:, :3] * scale
    a = o3d.geometry.PointCloud()
    a.points = o3d.utility.Vector3dVector(np.ascontiguousarray(p1))
    b = o3d.geometry.PointCloud()
    b.points = o3d.utility.Vector3dVector(np.ascontiguousarray(p2))
    if not args.no_save:
        o3d.io.write_point_cloud(os.path.join(scan_dir, f"{args.scan_id}_arc1.ply"), a.voxel_down_sample(args.voxel))
        o3d.io.write_point_cloud(os.path.join(scan_dir, f"{args.scan_id}_arc2.ply"), b.voxel_down_sample(args.voxel))
        print(f"  wrote {args.scan_id}_arc1.ply / _arc2.ply")

    a = a.voxel_down_sample(0.02)
    b = b.voxel_down_sample(0.02)
    reg = o3d.pipelines.registration.registration_icp(
        b, a, 0.2, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    T = reg.transformation
    yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    print(f"\narc2 -> arc1 ICP: fitness={reg.fitness:.3f} rmse={reg.inlier_rmse*1000:.1f}mm "
          f"yaw={yaw:+.3f} deg shift=({T[0,3]*1000:+.1f},{T[1,3]*1000:+.1f},{T[2,3]*1000:+.1f})mm")
    print("yaw ~ 0 => step angle correct; |yaw| proportional to azimuth error")


if __name__ == "__main__":
    main()
