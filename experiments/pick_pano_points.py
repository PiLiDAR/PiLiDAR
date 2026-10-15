"""Solve the LiDAR<->camera sphere rotation from hand-picked point pairs.

Cross-modal automatic matching (SIFT / NCC / MI) is unreliable between the RGB
panorama and the raw-intensity LiDAR panorama, so this lets **you** pick the
correspondences and solves the rotation exactly.

Both panoramas are 2:1 equirectangular and image the same scene from the same
nodal point, so the mapping between them is (to first order) a pure **rotation of
the sphere** -- the relative orientation of the LiDAR (cloud) frame and the camera
(RGB) frame. That rotation is exactly what `VERTEXCOLOUR.ROTATE_DEG` implements,
and with a perfect mount + no parallax it should be identity (and `CENTER_M`
zero), so this doubles as an independent check of `3D.CALIBRATION`.

How many pairs?
  1 pair  -> 2 constraints, 3 unknowns : under-determined (leaves roll free)
  2 pairs -> 4 constraints            : exact solve, no redundancy
  3+      -> well-conditioned least-squares (use 3 spread out, 5-6 to be safe)

Usage
-----
Interactive (two windows: click a feature in RGB, then its match in LiDAR,
repeat; keys:  c = compute,  r = reset,  q = quit)::

    ~/venv/bin/python experiments/pick_pano_points.py 261015-1839

From a text file (`x_rgb y_rgb x_lidar y_lidar` per line, pixels of the ORIGINAL
panorama sizes)::

    ~/venv/bin/python experiments/pick_pano_points.py 261015-1839 --pairs pairs.txt

The result is printed as `VERTEXCOLOUR.ROTATE_DEG = [rx, ry, rz]` (euler xyz, deg).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# --- pixel -> unit direction (equirect, both 2:1) ---------------------------
# LiDAR: matches lib.pointcloud.get_sampling_coordinates (so the solved rotation
# is directly the cloud->camera rotation to put into VERTEXCOLOUR.ROTATE_DEG).
def lidar_dir(x, y, W, H):
    # The rendered LiDAR panorama shares the RGB panorama's equirectangular
    # convention (both 2:1, x=0 -> -pi, centre -> 0, y=0 -> +pi/2).  Using the
    # SAME pixel->direction map for both makes the solved rotation the direct
    # image-to-image one (identity => the mount already aligns them).
    return rgb_dir(x, y, W, H)


# RGB: standard equirect (Hugin), x=0 -> -pi, centre -> 0; y=0 -> +pi/2 (up).
def rgb_dir(x, y, W, H):
    e = np.pi / 2 - np.pi * (y / H)
    phi = 2 * np.pi * (x / W) - np.pi
    return np.array([np.cos(e) * np.cos(phi), np.cos(e) * np.sin(phi), np.sin(e)])


def solve(pairs, size_rgb, size_lid):
    """pairs: list of ((xr,yr),(xl,yl)) in ORIGINAL pixel coords -> rotation info."""
    A, B = [], []
    for (xr, yr), (xl, yl) in pairs:
        B.append(rgb_dir(xr, yr, *size_rgb))
        A.append(lidar_dir(xl, yl, *size_lid))
    A, B = np.array(A), np.array(B)
    rot, rssd = R.align_vectors(B, A)               # R maps lidar dirs -> rgb dirs
    pred = rot.apply(A)
    ang = np.degrees(np.arccos(np.clip((pred * B).sum(1), -1, 1)))
    return rot, ang, rssd


def annotate(img, pts, color, hover=None, label=""):
    """Draw picked points + a synthetic crosshair that follows the mouse.

    The crosshair is drawn into the image on purpose: under WSLg the system
    pointer is often not rendered over an OpenCV (Qt/xcb) window, so the window
    shows no visible cursor. Painting our own marker makes the position obvious.
    """
    out = img.copy()
    for i, (x, y) in enumerate(pts):
        cv2.drawMarker(out, (int(x), int(y)), color, cv2.MARKER_CROSS, 30, 2)
        cv2.circle(out, (int(x), int(y)), 6, color, 2)
        cv2.putText(out, str(i), (int(x) + 10, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    if hover is not None:
        hx, hy = int(hover[0]), int(hover[1])
        cv2.line(out, (hx - 20, hy), (hx + 20, hy), (0, 255, 255), 1)
        cv2.line(out, (hx, hy - 20), (hx, hy + 20), (0, 255, 255), 1)
        cv2.circle(out, (hx, hy), 3, (0, 255, 255), -1)
        cv2.putText(out, f"{hx},{hy}", (hx + 12, hy + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    if label:
        cv2.putText(out, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    return out


class Picker:
    def __init__(self, rgb, lid, win):
        self.rgb, self.lid, self.win = rgb, lid, win
        self.rgb_pts, self.lid_pts = [], []
        self.hover = {"rgb": None, "lidar": None}
        self.expect = "rgb"

    def _cb(self, event, x, y, flags, param):
        side = param
        if event == cv2.EVENT_MOUSEMOVE:
            self.hover[side] = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            if side != self.expect:
                print(f"  [click the {self.expect.upper()} window next]")
                return
            (self.rgb_pts if side == "rgb" else self.lid_pts).append((x, y))
            print(f"  pair {len(self.rgb_pts)}: {side} ({x},{y})")
            self.expect = "lidar" if side == "rgb" else "rgb"

    def run(self):
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win, self._cb, "rgb")
        cv2.namedWindow(self.win + "_lidar", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.win + "_lidar", self._cb, "lidar")
        print("click a feature in RGB, then its match in LiDAR; repeat.")
        print("keys: c = compute, r = reset, q = quit\n")
        while True:
            rgb_lbl = "RGB  <-- click here" if self.expect == "rgb" else "RGB"
            lid_lbl = "LIDAR  <-- click here" if self.expect == "lidar" else "LIDAR"
            cv2.imshow(self.win, annotate(self.rgb, self.rgb_pts, (0, 0, 255),
                                          self.hover["rgb"], rgb_lbl))
            cv2.imshow(self.win + "_lidar", annotate(self.lid, self.lid_pts, (0, 255, 0),
                                                     self.hover["lidar"], lid_lbl))
            k = cv2.waitKey(20) & 0xFF
            if k == ord("q"):
                break
            if k == ord("r"):
                self.rgb_pts, self.lid_pts, self.expect = [], [], "rgb"
                print("  reset")
            if k == ord("c"):
                if len(self.rgb_pts) >= 2 and len(self.rgb_pts) == len(self.lid_pts):
                    return list(zip(self.rgb_pts, self.lid_pts))
                print("  need >= 2 paired clicks (equal counts)")
        cv2.destroyAllWindows()
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--rgb", default=None)
    ap.add_argument("--lidar", default=None)
    ap.add_argument("--pairs", default=None, help="text file: x_rgb y_rgb x_lidar y_lidar per line")
    ap.add_argument("--scale", type=float, default=0.5, help="display downscale factor")
    args = ap.parse_args()

    scan = os.path.join(ROOT, "scans", args.scan_id)
    rgb_path = args.rgb or os.path.join(scan, f"{args.scan_id}_blended_fused.jpg")
    lid_path = args.lidar or os.path.join(scan, f"{args.scan_id}_lidar.jpg")
    rgb0 = cv2.imread(rgb_path, cv2.IMREAD_GRAYSCALE)
    lid0 = cv2.imread(lid_path, cv2.IMREAD_GRAYSCALE)
    if rgb0 is None or lid0 is None:
        raise SystemExit(f"cannot read {rgb_path} / {lid_path}")
    Hr, Wr = rgb0.shape
    Hl, Wl = lid0.shape
    s = args.scale
    rgb = cv2.resize(rgb0, (int(Wr * s), int(Hr * s)), interpolation=cv2.INTER_AREA)
    lid = cv2.resize(lid0, (int(Wl * s), int(Hl * s)), interpolation=cv2.INTER_AREA)

    if args.pairs:
        raw = np.loadtxt(args.pairs)
        pairs = [((xr, yr), (xl, yl)) for xr, yr, xl, yl in np.atleast_2d(raw)]
    else:
        pairs = Picker(rgb, lid, f"{args.scan_id} RGB").run()
        if not pairs:                                     # convert display px -> original px
            return
        pairs = [((xr / s, yr / s), (xl / s, yl / s)) for (xr, yr), (xl, yl) in pairs]

    rot, ang, rssd = solve(pairs, (Wr, Hr), (Wl, Hl))
    e = rot.as_euler("xyz", degrees=True)
    print(f"\n{len(pairs)} pairs")
    print(f"  per-pair residual (deg): {np.round(ang, 2).tolist()}")
    print(f"  rms residual:            {ang.std():.2f} deg (max {ang.max():.2f})")
    print(f"\n  ROTATE_DEG (euler xyz) = [{e[0]:.3f}, {e[1]:.3f}, {e[2]:.3f}]")
    print(f"  (identity => mount already aligned; CENTER_M would then be ~0)")

    # annotated overlay for a sanity check (resize the LiDAR pane to the RGB pane)
    lw = rgb.shape[1]
    lh = int(round(lid.shape[0] * lw / lid.shape[1]))
    rgb_an = cv2.cvtColor(annotate(rgb, [(x * s, y * s) for (x, y), _ in pairs], (0, 0, 255)),
                          cv2.COLOR_GRAY2BGR)
    lid_an = cv2.cvtColor(annotate(cv2.resize(lid, (lw, lh)),
                                   [(x * s, y * s) for _, (x, y) in pairs], (0, 255, 0)),
                          cv2.COLOR_GRAY2BGR)
    ov = np.vstack([rgb_an, lid_an])
    out = os.path.join(scan, "tmp", f"{args.scan_id}_pano_pairs.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, ov)
    print(f"  annotated pairs -> {os.path.relpath(out, ROOT)}")


if __name__ == "__main__":
    main()