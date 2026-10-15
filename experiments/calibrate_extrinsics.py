"""Calibrate the LiDAR <-> camera panoramic extrinsics from a scan.

Recovers the projection parameters that make the synthetic LiDAR panorama (grey,
laser intensity) line up with the stitched colour panorama, so the *same*
projection can then colourise the point cloud accurately.

The projection used for both the LiDAR panorama and vertex colouring is

    p' = R_euler_xyz(rx, ry, rz) @ (p - centre)       # centre in cloud units (m)
    (theta, phi) = spherical(p')                      # -> equirect (theta, phi)

so the unknowns are the **projection centre** (the effective fisheye nodal point,
which subsumes the vertical offset) plus the rotation between the LiDAR/object
frame and the panorama. For this camera (facing **+Y**, up +Z):

  * **rx** — **pitch** (moves the horizon = a vertical shift)
  * **ry** — **roll** about the camera's facing axis (rotates the view)
  * **rz** — **yaw / pan** (horizontal shift)

These are *projection-time* parameters — they do not touch the cloud assembly
(`Y_OFFSET`/`LIDAR_OFFSET_ANGLE`), which is a separate geometric calibration.

Method: multi-scale edge alignment. Both panoramas are reduced to gradient-
magnitude maps (they are different modalities — laser intensity vs. RGB — so raw
pixel correlation is meaningless); the objective is the masked, zero-mean
normalised cross-correlation of those maps. A coarse grid on (roll, yaw) seeds a
Nelder-Mead refinement over all six parameters.

Usage (desktop; needs the scan's raw container + colour pano):
    ~/venv/bin/python experiments/calibrate_extrinsics.py 261012-0037
    ~/venv/bin/python experiments/calibrate_extrinsics.py 261012-0037 --mirror
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from scipy.optimize import minimize  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from lib.pointcloud import (  # noqa: E402
    _as_uint8_intensity, get_sampling_coordinates, angular_from_cartesian,
    load_raw_scan, merge_2D_points,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# working resolution for the objective. Must be small enough that the subsampled
# points FILL the pixels — a sparse render is speckly and does not correlate.
WRK_W, WRK_H = 720, 360
OBJ_POINTS = 1_200_000    # enough points to fill WRK_W x WRK_H densely


def load_points(cfg, raw):
    """Assemble the cloud in the same frame the exporter uses (m, mount applied)."""
    pos, rot = cfg["3D"]["CALIBRATION"]["POSITION"], cfg["3D"]["CALIBRATION"]["ROTATION"]
    scale = cfg["3D"]["SCALE"]

    p3 = merge_2D_points(raw, mount=(pos, rot))
    pts = p3[:, :3].astype(np.float64) * scale
    return pts, p3[:, 3]


def gradient_map(gray, w, h):
    """Blur -> resize -> Sobel magnitude, normalized to [0, 1]."""
    g = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), 2.0)
    g = cv2.resize(g, (w, h), interpolation=cv2.INTER_AREA)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    m = mag.max()
    return mag / m if m > 0 else mag


def project_pano(pts, intensity, params, w, h, valid_only=True):
    """Render the LiDAR intensity panorama under the current projection params.

    ``params`` = (cx, cy, cz, rx, ry, rz): centre (m) + euler xyz (deg). The centre
    is computed in metres here (the cloud is meter-scaled); the emitted config
    snippet converts it to ``VERTEXCOLOUR.CENTER_MM`` (mm) + ``ROTATE_DEG``.
    Returns (gray_uint8, valid_mask).
    """
    cx, cy, cz, rx, ry, rz = params
    rot = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    p = (pts - np.array([cx, cy, cz])) @ rot.T
    ang = angular_from_cartesian(p)
    x, y = get_sampling_coordinates(ang, (h, w))
    gray = np.zeros((h, w), np.uint8)
    mask = np.zeros((h, w), np.uint8)
    gray[y, x] = _as_uint8_intensity(intensity)
    mask[y, x] = 1
    if valid_only:
        gray = cv2.medianBlur(gray, 3)
    return gray, mask


def ncc_objective(a, b, mask):
    """Negative zero-mean NCC of two gradient maps over ``mask`` (to minimize).

    Vectorized (elementwise passes, no boolean fancy-indexing) for speed.
    """
    m = mask
    n = int(m.sum())
    if n < 100:
        return 1.0
    a_m = np.where(m, a, 0.0)
    b_m = np.where(m, b, 0.0)
    sa = a_m.sum()
    sb = b_m.sum()
    cov = float((a_m * b_m).sum()) - sa * sb / n
    va = float((a_m * a_m).sum()) - sa * sa / n
    vb = float((b_m * b_m).sum()) - sb * sb / n
    denom = np.sqrt(va * vb)
    if denom < 1e-9:
        return 1.0
    return -float(cov / denom)


def subsample(pts, intensity, n):
    if len(pts) <= n:
        return pts, intensity
    idx = np.linspace(0, len(pts) - 1, n).astype(int)
    return pts[idx], intensity[idx]


def build_reference(ref_gray, w, h):
    """Gradient map + validity mask of the colour panorama at (w, h)."""
    ref_edge = gradient_map(ref_gray, w, h)
    valid = cv2.resize((ref_gray > 8).astype(np.float32), (w, h),
                       interpolation=cv2.INTER_NEAREST) > 0.5
    return ref_edge, valid


def make_cost(pts, intensity, w, h, ref_edge, ref_valid):
    def cost(params):
        gray, mask = project_pano(pts, intensity, params, w, h)
        edge = gradient_map(gray, w, h)
        return ncc_objective(edge, ref_edge, mask.astype(bool) & ref_valid)
    return cost


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--mirror", action="store_true",
                    help="the two panoramas turn in opposite directions")
    ap.add_argument("--out-dir", default=None,
                    help="where to write debug images (default: scans/<id>/tmp/)")
    args = ap.parse_args()

    cfg = json.load(open(os.path.join(ROOT, "config.json")))
    scan_dir = os.path.join(ROOT, "scans", args.scan_id)
    raw = load_raw_scan(os.path.join(scan_dir, f"{args.scan_id}_lidar.pkl"))
    pts, intensity = load_points(cfg, raw)
    print(f"cloud: {len(pts)} points")

    ref = cv2.imread(os.path.join(scan_dir, f"{args.scan_id}{cfg['PANO']['OUTPUT_NAME']}"))
    if ref is None:
        sys.exit("colour panorama not found")
    ref_gray = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
    if args.mirror:
        ref_gray = ref_gray[:, ::-1]   # the two panoramas turn in opposite senses

    # --- stage 1: coarse roll/yaw on a low-res gradient map (fast) ---
    CW, CH = WRK_W // 3, WRK_H // 3
    cpts, cI = subsample(pts, intensity, OBJ_POINTS)
    ref_edge_c, ref_valid_c = build_reference(ref_gray, CW, CH)
    cost_c = make_cost(cpts, cI, CW, CH, ref_edge_c, ref_valid_c)
    print("coarse search over roll(ry)/yaw(rz) ...")
    best = None
    for ry in np.arange(-20, 20, 5):
        for rz in np.arange(-180, 180, 10):
            c = cost_c((0.0, 0.0, 0.0, 0.0, ry, rz))
            if best is None or c < best[0]:
                best = (c, (0.0, 0.0, 0.0, 0.0, ry, rz))
    print(f"  init cost {best[0]:.4f}  ry={best[1][4]:.1f} rz={best[1][5]:.1f}")

    # --- stage 2: refine all six params at working resolution ---
    fpts, fI = subsample(pts, intensity, OBJ_POINTS)
    ref_edge, ref_valid = build_reference(ref_gray, WRK_W, WRK_H)
    cost = make_cost(fpts, fI, WRK_W, WRK_H, ref_edge, ref_valid)
    print(f"Nelder-Mead refinement ({len(fpts)} pts, {WRK_W}x{WRK_H}) ...")
    res = minimize(cost, best[1], method="Nelder-Mead",
                   options={"maxiter": 3000, "xatol": 1e-4, "fatol": 1e-7})
    cx, cy, cz, rx, ry, rz = res.x
    print(f"\ncost {res.fun:.4f}")
    print(f"  centre (m) : [{cx:.4f}, {cy:.4f}, {cz:.4f}]  "
          f"({cx*1000:.1f}, {cy*1000:.1f}, {cz*1000:.1f} mm)")
    print(f"  rotate xyz (deg) [pitch, roll, yaw]: [{rx:.2f}, {ry:.2f}, {rz:.2f}]")

    # --- overlay for visual verification ---
    gray, mask = project_pano(fpts, fI, res.x, WRK_W, WRK_H)
    edge = gradient_map(gray, WRK_W, WRK_H)
    overlay = np.zeros((WRK_H, WRK_W, 3), np.uint8)
    overlay[..., 2] = (ref_valid * (ref_edge > 0.15) * 255).astype(np.uint8)   # red = colour edges
    overlay[..., 1] = ((mask.astype(bool) & (edge > 0.15)) * 255).astype(np.uint8)  # green = lidar edges
    out_dir = args.out_dir or os.path.join(scan_dir, "tmp")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{args.scan_id}_calib_overlay_coarse.png")
    cv2.imwrite(out, overlay)
    print(f"\noverlay (red=colour, green=lidar, yellow=match) -> {out}")

    # --- also store the two input images (and their gradient maps) for inspection ---
    #
    # Render at a resolution the points actually FILL (the raw cloud is ~2M points,
    # so 1440x720 is ~2 pts/px). Rendering at the colour pano's 3600x1800 would be
    # ~0.3 pts/px -> a mostly-black, speckly image.
    img_dir = out_dir

    def _u8(x):
        x = np.asarray(x, np.float32)
        x = x - x.min()
        return (x / (x.max() + 1e-9) * 255).astype(np.uint8)

    save_w, save_h = 1440, 720
    lidar_pano, lidar_mask = project_pano(pts, intensity, res.x, save_w, save_h)
    # also the ASSEMBLY pano (no calibrated rotation) so the fitted rotation is visible
    lidar_raw, _ = project_pano(pts, intensity, (0, 0, 0, 0, 0, 0), save_w, save_h)
    # the reference AS USED (already flipped when --mirror), resized to match
    ref_pano = cv2.resize(ref_gray, (save_w, save_h), interpolation=cv2.INTER_AREA)
    lidar_edges = gradient_map(lidar_pano, save_w, save_h)
    ref_edges = gradient_map(ref_pano, save_w, save_h)

    overlay_save = np.zeros((save_h, save_w, 3), np.uint8)
    overlay_save[..., 2] = (((ref_pano > 8) & (ref_edges > 0.15)) * 255).astype(np.uint8)   # red
    overlay_save[..., 1] = (((lidar_mask > 0) & (lidar_edges > 0.15)) * 255).astype(np.uint8)  # green

    def _save(name, img):
        cv2.imwrite(os.path.join(img_dir, f"{args.scan_id}_calib_{name}.png"), img)

    _save("lidar_intensity", lidar_pano)
    _save("lidar_intensity_raw", lidar_raw)
    _save("colour_gray", ref_pano)
    _save("lidar_edges", _u8(lidar_edges))
    _save("colour_edges", _u8(ref_edges))
    _save("overlay", overlay_save)
    stack = np.vstack([cv2.cvtColor(lidar_pano, cv2.COLOR_GRAY2BGR),
                       np.full((4, save_w, 3), 255, np.uint8),
                       cv2.cvtColor(ref_pano, cv2.COLOR_GRAY2BGR)])
    _save("stack", stack)
    print(f"saved input images (lidar/colour + edges + overlay) -> {img_dir}/")

    params = {"CENTER_MM": [round(cx * 1000, 3), round(cy * 1000, 3), round(cz * 1000, 3)],
              "ROTATE_DEG": [round(rx, 3), round(ry, 3), round(rz, 3)],
              "mirror": bool(args.mirror), "cost": float(res.fun)}
    print("\nconfig snippet:\n" + json.dumps({"VERTEXCOLOUR": params}, indent=2))


if __name__ == "__main__":
    main()
