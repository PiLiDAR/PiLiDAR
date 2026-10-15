"""Render a coloured point cloud offscreen (headless) to inspect texture alignment.

    ~/venv/bin/python experiments/render_cloud.py scans/261015-1839/261015-1839.ply
    ~/venv/bin/python experiments/render_cloud.py <ply> --yaw 0 --pitch 0 --out /tmp/x.png
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ply")
    ap.add_argument("--out", default=None)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=0.0)
    ap.add_argument("--dist", type=float, default=0.0)
    ap.add_argument("--size", type=int, nargs=2, default=[1600, 900])
    ap.add_argument("--max-range", type=float, default=0.0,
                    help="drop points farther than this from the centre (m)")
    args = ap.parse_args()

    pcd = o3d.io.read_point_cloud(args.ply)
    pts = np.asarray(pcd.points)
    cols = np.asarray(pcd.colors)
    center = pts.mean(0)
    if args.max_range:
        keep = np.linalg.norm(pts - center, axis=1) < args.max_range
        pts, cols = pts[keep], cols[keep]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.colors = o3d.utility.Vector3dVector(cols)
    extent = float(np.linalg.norm(pts.max(0) - pts.min(0)))
    dist = args.dist or extent * 0.9
    print(f"{len(pts)} points, center {np.round(center,3).tolist()}, extent {extent:.2f} m")

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False, width=args.size[0], height=args.size[1])
    vis.add_geometry(pcd)
    opt = vis.get_render_option()
    opt.point_size = 1.5
    opt.background_color = np.array([0.05, 0.05, 0.05])
    opt.light_on = False                     # unlit -> show the baked colours

    ctr = vis.get_view_control()
    yaw, pitch = np.radians(args.yaw), np.radians(args.pitch)
    eye = center + dist * np.array([np.cos(pitch) * np.sin(yaw),
                                    -np.cos(pitch) * np.cos(yaw),
                                    np.sin(pitch)])
    ctr.set_lookat(center)
    ctr.set_up([0, 0, 1])
    ctr.set_front((eye - center) / np.linalg.norm(eye - center))
    ctr.set_zoom(0.35)
    vis.poll_events(); vis.update_renderer()
    out = args.out or os.path.join(os.path.dirname(args.ply), "cloud_render.png")
    vis.capture_screen_image(out, do_render=True)
    vis.destroy_window()
    print("wrote", out)


if __name__ == "__main__":
    main()