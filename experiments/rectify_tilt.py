"""Probe a tilt ("x rotation") of the LiDAR unit in scanner space.

Visual clue: vertical structures (door frames) come out **diagonal**, and the two
fan halves lean and their panoramas disagree. A pure viewpoint offset (parallax)
cannot do that - a vertical line stays vertical in any equirectangular panorama
under translation. A **tilt of the LiDAR disc** does, because it maps a fixed world
azimuth to a height-dependent one.

The tilt physically sits in the mount rotation (euler xyz) that
:func:`merge_2D_points` applies *before* the revolve, i.e. in the fan's rotating
frame - exactly like a LiDAR unit mounted a little off-square. So we sweep the
mount **x** component (and optionally the roll **y**), reassemble, render an
intensity panorama and score how much edge energy is **vertical** (walls) vs
horizontal. The best tilt is the one that squares the walls up.

Run separately for the two fan halves (``--half left/right``) to see whether they
want the *same* tilt (a real world-frame tilt) or *opposite* tilt (a relative
error between the halves).

    ~/venv/bin/python experiments/rectify_tilt.py 261015-1726
    ~/venv/bin/python experiments/rectify_tilt.py 261015-1726 --half left
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from lib.pointcloud import load_raw_scan, merge_2D_points, render_lidar_pano  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W, H = 1440, 720


def assemble(raw, pos, rot, half=None):
    """Assemble the cloud (optionally one fan half) with mount (pos, rot)."""
    if half is None:
        sub = raw
    else:
        positive = half == "right"
        fans = [np.asarray(f)[np.asarray(f)[:, 0] >= 0 if positive else np.asarray(f)[:, 0] < 0]
                for f in raw["cartesian"]]
        sub = {"z_angles": raw["z_angles"], "cartesian": fans}
    return merge_2D_points(sub, mount=(pos, rot))[:, :3], None


def verticality(points, intensity):
    """Length-weighted verticality of straight edges (1.0 = all edges upright).

    Uses Hough line segments on the panorama edges and measures how close each is
    to vertical (column direction), so a tilted LiDAR (diagonal door frames) scores
    low and the rectifying tilt scores highest.
    """
    pano = render_lidar_pano(points, intensity, W, H, center=(0, 0, 0))
    g = cv2.GaussianBlur(pano, (0, 0), 1.0)
    edges = cv2.Canny(g, 40, 120)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40,
                            minLineLength=80, maxLineGap=12)
    if lines is None:
        return 0.0
    num, den = 0.0, 0.0
    for x0, y0, x1, y1 in np.asarray(lines).reshape(-1, 4):
        dx, dy = x1 - x0, y1 - y0
        length = np.hypot(dx, dy)
        # angle from VERTICAL (col axis); 0 = perfectly upright
        ang = np.degrees(np.arctan2(dx, dy))
        if abs(ang) > 35:
            continue
        num += length * np.cos(np.radians(ang))**2
        den += length
    return float(num / den) if den else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--half", choices=["left", "right", "both", "none"], default="none")
    ap.add_argument("--x", type=float, nargs=3, metavar=("LO", "HI", "STEP"),
                    default=[-6, 6, 1.0], help="mount x (pitch) sweep in deg")
    ap.add_argument("--y", type=float, default=None, help="fixed extra roll trim (deg) added to mount y")
    ap.add_argument("--render-only", action="store_true", help="just save panoramas, no scoring")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    pos = cfg["3D"]["CALIBRATION"]["POSITION"]
    rot0 = list(cfg["3D"]["CALIBRATION"]["ROTATION"])
    raw = load_raw_scan(os.path.join(ROOT, "scans", args.scan_id, f"{args.scan_id}_lidar.pkl"))

    # keep the raw intensity (col 3) alongside
    def split_intensity(half):
        if half is None:
            return raw, np.concatenate([np.asarray(f)[:, 2] for f in raw["cartesian"]])
        positive = half == "right"
        fans, intens = [], []
        for f in raw["cartesian"]:
            f = np.asarray(f)
            m = f[:, 0] >= 0 if positive else f[:, 0] < 0
            fans.append(f[m])
            intens.append(f[m, 2])
        return {"z_angles": raw["z_angles"], "cartesian": fans}, np.concatenate(intens)

    order = {"none": [None], "left": ["left"], "right": ["right"], "both": ["left", "right"]}[args.half]

    lo, hi, step = args.x
    xs = np.arange(lo, hi + 1e-9, step)
    print(f"scan {args.scan_id}: mount x sweep {list(np.round(xs,2))} deg "
          f"(base rot {rot0}), halves={order}\n")

    results = {}
    for half in order:
        sub, intens = split_intensity(half)
        if args.render_only:
            for x in xs:
                rot = [float(x), rot0[1] + (args.y or 0.0), rot0[2]]
                p3 = merge_2D_points(sub, mount=(pos, rot))[:, :3]
                pano = render_lidar_pano(p3, intens, W, H, center=(0, 0, 0))
                out = os.path.join(ROOT, "scans", args.scan_id, "tmp",
                                   f"rectify_{half or 'both'}_x{x:+.1f}.png")
                os.makedirs(os.path.dirname(out), exist_ok=True)
                cv2.imwrite(out, pano)
                print(f"  rendered {os.path.relpath(out, ROOT)}")
            continue
        scores = []
        for x in xs:
            rot = [float(x), rot0[1] + (args.y or 0.0), rot0[2]]
            p3 = merge_2D_points(sub, mount=(pos, rot))[:, :3]
            s = verticality(p3, intens)
            scores.append(s)
            print(f"  [{str(half):5s}] x={x:+5.2f}deg  verticality={s:.4f}")
        scores = np.array(scores)
        best = xs[int(scores.argmax())]
        results[half] = (best, scores.max())
        print(f"  -> best x = {best:+.2f} deg (verticality {scores.max():.4f})\n")

    print("summary (tilt that best squares the walls):")
    for half, (b, s) in results.items():
        print(f"  {str(half):5s}: x = {b:+.2f} deg  score {s:.4f}")
    if "left" in results and "right" in results:
        dl, dr = results["left"][0], results["right"][0]
        rel = dl - dr
        print(f"\nleft vs right best-x differ by {rel:+.2f} deg "
              f"({'- relative/opposite error' if abs(rel) > 0.5 else 'consistent world-frame tilt'})")


if __name__ == "__main__":
    main()