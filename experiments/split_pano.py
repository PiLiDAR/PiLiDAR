"""Render the LiDAR intensity panorama split into its two fan halves.

Each fan's two in-plane radial halves (x<0 / x>0) both sweep the full 360 deg, so
the sweep actually contains **two overlapping 360 panoramas** seen from two
slightly different positions (the ~2x offset parallax). Rendered as one pano they
mix, which is why the intensity pano shows sudden brightness changes (where one
half is nearer).

This renders a LEFT-half and a RIGHT-half panorama (and overlays them) so their
alignment can be checked directly in 2D.

    ~/venv/bin/python experiments/split_pano.py 261015-1726
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402

from lib.pointcloud import (  # noqa: E402
    load_raw_scan, merge_2D_points, render_lidar_pano,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 2048, 1024


def split_half(raw, positive):
    fans = []
    for fan in raw["cartesian"]:
        fan = np.asarray(fan)
        c = fan[:, 0]
        fans.append(fan[c >= 0] if positive else fan[c < 0])
    return {"z_angles": raw["z_angles"], "cartesian": fans}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    mount = (cfg["3D"]["CALIBRATION"]["POSITION"], cfg["3D"]["CALIBRATION"]["ROTATION"])
    scan_dir = os.path.join(ROOT, "scans", args.scan_id)
    raw = load_raw_scan(os.path.join(scan_dir, f"{args.scan_id}_lidar.pkl"))

    panos = {}
    for name, positive in (("left", False), ("right", True)):
        sub = split_half(raw, positive)
        p3 = merge_2D_points(sub, mount=mount)
        panos[name] = render_lidar_pano(p3[:, :3], p3[:, 3], W, H)
        print(f"{name:5s} half: {len(p3)} pts")

    # edge overlay: red = left edges, green = right edges
    def edges(g):
        gg = cv2.GaussianBlur(g.astype(np.float32), (0, 0), 2.0)
        gx = cv2.Sobel(gg, cv2.CV_32F, 1, 0)
        gy = cv2.Sobel(gg, cv2.CV_32F, 0, 1)
        m = np.hypot(gx, gy)
        return m / (m.max() + 1e-9)

    el, er = edges(panos["left"]), edges(panos["right"])
    ov = np.zeros((H, W, 3), np.uint8)
    ov[..., 2] = (el > 0.15) * 255     # red = left
    ov[..., 1] = (er > 0.15) * 255     # green = right

    # 2-D alignment: best horizontal (azimuth) shift between the two edge maps
    def ncc(a, b):
        m = (a > 0.1) & (b > 0.1)
        if m.sum() < 100:
            return -2
        av = a[m] - a[m].mean(); bv = b[m] - b[m].mean()
        d = np.sqrt((av * av).sum() * (bv * bv).sum())
        return float((av * bv).sum() / d) if d > 0 else -2

    def best_shift(a, b, rng=None):
        rng = rng or (W // 8)
        bb = (-2, 0)
        for shift in range(-rng, rng):
            c = ncc(np.roll(a, shift, axis=1), b)
            if c > bb[0]:
                bb = (c, shift)
        return bb

    c, shift = best_shift(el, er)
    deg = shift / W * 360.0
    print(f"\nleft vs right pano: best azimuth shift = {shift} px = {deg:+.2f} deg "
          f"(NCC {c:.3f})")

    # per-elevation band: is the offset constant (yaw) or range/direction dependent?
    print("\n  elevation band -> best azimuth shift:")
    for i in range(8):
        y0, y1 = i * H // 8, (i + 1) * H // 8
        cc, sh = best_shift(el[y0:y1], er[y0:y1], W // 16)
        print(f"    rows {y0:4d}-{y1:4d}: {sh * 360.0 / W:+6.2f} deg (NCC {cc:.3f})")

    out = os.path.join(scan_dir, "tmp")
    os.makedirs(out, exist_ok=True)
    cv2.imwrite(os.path.join(out, f"{args.scan_id}_pano_left.png"), panos["left"])
    cv2.imwrite(os.path.join(out, f"{args.scan_id}_pano_right.png"), panos["right"])
    cv2.imwrite(os.path.join(out, f"{args.scan_id}_pano_edges_overlay.png"), ov)
    stack = np.vstack([cv2.cvtColor(panos["left"], cv2.COLOR_GRAY2BGR),
                       np.full((4, W, 3), 255, np.uint8),
                       cv2.cvtColor(panos["right"], cv2.COLOR_GRAY2BGR)])
    cv2.imwrite(os.path.join(out, f"{args.scan_id}_pano_stack.png"), stack)
    print(f"saved left/right panos + overlay + stack -> {out}/")


if __name__ == "__main__":
    main()