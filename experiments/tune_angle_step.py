"""Interactively tune the per-step rotation angle of a (360 deg) sweep.

Each LiDAR plane is revolved by the *stepper* angle ``z`` when the cloud is
assembled. If the true angular scale is slightly off (gear ratio / microsteps /
anchor miscount), every plane is rotated by the wrong amount and the error grows
with angle, so the assembled cloud "fans out" instead of closing. Because a 360
deg sweep sees every surface from both sides, walls should form *single* sheets.

This rebuilds the cloud with a corrected mapping and lets you nudge it live:

    angle_used = SCALE * z + OFFSET            (degrees)

``SCALE`` is the key knob. Find the value that makes doubled/fanned walls merge
into one sheet.

Usage (desktop; interactive open3d window):
    ~/venv/bin/python experiments/tune_angle_step.py 261014-1954
    ~/venv/bin/python experiments/tune_angle_step.py 261014-1954 --scale 0.999

Keys:
    ]  / [   SCALE   +/- 0.005 %   (fine)
    }  / {   SCALE   +/- 0.05  %   (coarse)
    .  / ,   OFFSET  +/- 0.02 deg
    R  reset to the start values       P  print current SCALE/OFFSET
    q  quit
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.pointcloud import load_raw_scan, remove_NaN  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PREVIEW_POINTS = 800_000      # subsample once; the visual check does not need all 4M


def assemble_local(raw, cfg):
    """Stacked per-plane local 3D points + their plane index, before the revolve.

    Mirrors ``merge_2D_points`` up to (but not including) the z-revolve, so only
    the revolve angle changes when SCALE/OFFSET change. Vectorised (no per-plane
    open3d objects) so a rebuild is fast enough for interactive tuning.
    """
    ox, oy, oz = 0.0, cfg["3D"]["Y_OFFSET"], 0.0
    ao = np.radians(cfg["LIDAR"]["LIDAR_OFFSET_ANGLE"])
    ca, sa = np.cos(ao), np.sin(ao)

    chunks, idx = [], []
    for i, fan in enumerate(raw["cartesian"]):
        pts = np.insert(np.asarray(fan, dtype=float), 1, values=0.0, axis=1)   # (n,4)
        chunks.append(pts)
        idx.append(np.full(len(pts), i))
    P = np.vstack(chunks)
    pidx = np.concatenate(idx)

    # rotate about local Y by angle_offset: x' = x ca + z sa ; z' = -x sa + z ca
    x, z = P[:, 0].copy(), P[:, 2].copy()
    P[:, 0] = x * ca + z * sa
    P[:, 2] = -x * sa + z * ca
    P[:, 0] += ox
    P[:, 1] += oy
    P[:, 2] += oz
    return P, pidx


def revolve(P, pidx, z_angles, scale, offset):
    """Revolve each point by ``-(SCALE*z[plane] + OFFSET)`` about Z (vectorised)."""
    ang = -np.radians(z_angles[pidx] * scale + offset)
    ca, sa = np.cos(ang), np.sin(ang)
    x, y = P[:, 0], P[:, 1]
    out = np.empty_like(P)
    out[:, 0] = x * ca - y * sa
    out[:, 1] = x * sa + y * ca
    out[:, 2] = P[:, 2]
    out[:, 3] = P[:, 3]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--stride", type=int, default=10,
                    help="use every Nth slice for a fast first impression (default 10)")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--offset", type=float, default=0.0)
    ap.add_argument("--scale-xy", type=float, default=0.001, help="mm -> m for the viewer")
    ap.add_argument("--voxel", type=float, default=0.01, help="voxel size (m) for --no-view file output")
    ap.add_argument("--no-view", action="store_true")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    scan_dir = os.path.join(ROOT, "scans", args.scan_id)
    raw = load_raw_scan(os.path.join(scan_dir, f"{args.scan_id}_lidar.pkl"))
    z_angles = np.asarray(raw["z_angles"], dtype=float)
    cartesian = list(raw["cartesian"])
    if args.stride > 1:
        z_angles = z_angles[::args.stride]
        cartesian = cartesian[::args.stride]
        raw = {"z_angles": z_angles, "cartesian": cartesian}
    P, pidx = assemble_local(raw, cfg)
    if len(P) > PREVIEW_POINTS:
        sel = np.random.default_rng(0).choice(len(P), PREVIEW_POINTS, replace=False)
        P, pidx = P[sel], pidx[sel]
    print(f"{len(z_angles)} planes (stride {args.stride}), preview {len(P)} points; "
          f"start scale={args.scale} offset={args.offset}")

    state = {"scale": args.scale, "offset": args.offset}

    def build(voxel=None):
        pts = revolve(P, pidx, z_angles, state["scale"], state["offset"])[:, :3] * args.scale_xy
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(remove_NaN(pts))
        if voxel:
            pcd = pcd.voxel_down_sample(voxel)
        return pcd

    if args.no_view:
        out = os.path.join("/tmp", f"tune_{args.scan_id}_{state['scale']:.5f}.ply")
        o3d.io.write_point_cloud(out, build(args.voxel))
        print("wrote", out)
        return

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"tune {args.scan_id}", width=1280, height=800)
    pcd = build(None)                       # no voxel: the 800k preview renders fine
    vis.add_geometry(pcd)

    def refresh():
        new = build(None)
        pcd.points = new.points
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        print(f"  scale={state['scale']:.6f}  offset={state['offset']:.4f} deg")

    def nudge_scale(f):
        state["scale"] *= f
        refresh()
        return False

    def nudge_offset(d):
        state["offset"] += d
        refresh()
        return False

    def reset(v):
        state["scale"], state["offset"] = args.scale, args.offset
        refresh()
        return False

    def show(v):
        print(f"\nCURRENT  scale={state['scale']:.6f}  offset={state['offset']:.4f} deg\n")
        return False

    vis.register_key_callback(ord("]"), lambda v: nudge_scale(1.00005))
    vis.register_key_callback(ord("["), lambda v: nudge_scale(1.0 / 1.00005))
    vis.register_key_callback(ord("}"), lambda v: nudge_scale(1.0005))
    vis.register_key_callback(ord("{"), lambda v: nudge_scale(1.0 / 1.0005))
    vis.register_key_callback(ord("."), lambda v: nudge_offset(0.02))
    vis.register_key_callback(ord(","), lambda v: nudge_offset(-0.02))
    vis.register_key_callback(ord("R"), reset)
    vis.register_key_callback(ord("P"), show)

    print("\nkeys: ] [ fine scale   } { coarse scale   . , offset   R reset   P print   q quit\n")
    vis.run()
    vis.destroy_window()
    print(f"\nFINAL  scale={state['scale']:.6f}  offset={state['offset']:.4f} deg")


if __name__ == "__main__":
    main()
