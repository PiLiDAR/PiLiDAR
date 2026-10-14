"""Calibrate the LiDAR <-> camera panoramic extrinsics from a scan.

Recovers the projection parameters that make the synthetic LiDAR panorama (grey,
laser intensity) line up with the stitched colour panorama, so the *same*
projection can then colourise the point cloud accurately.

The projection used for both the LiDAR panorama and vertex colouring is

    p' = R(yaw, pitch, roll) @ (p - centre)          # centre in cloud units (m)
    (theta, phi) = spherical(p')                      # -> equirect (theta, phi)

so the unknowns are the **projection centre** (the effective fisheye nodal point,
which subsumes the vertical offset) plus yaw/pitch/roll between the LiDAR frame
and the panorama. These are *projection-time* parameters — they do not touch the
cloud assembly (`Y_OFFSET`/`LIDAR_OFFSET_ANGLE`), which is a separate geometric
calibration.

Method: multi-scale edge alignment. Both panoramas are reduced to gradient-
magnitude maps (they are different modalities — laser intensity vs. RGB — so raw
pixel correlation is meaningless); the objective is the masked, zero-mean
normalised cross-correlation of those maps. A coarse grid on (yaw, pitch) seeds a
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
    """Assemble the cloud in the same frame the exporter uses (m, Z_OFFSET applied)."""
    y_offset = cfg["3D"]["Y_OFFSET"]
    z_offset = cfg["3D"]["Z_OFFSET"]
    scale = cfg["3D"]["SCALE"]
    ang_offset = cfg["LIDAR"]["LIDAR_OFFSET_ANGLE"]

    p3 = merge_2D_points(raw, position_offset=(0, y_offset, 0), angle_offset=ang_offset)
    pts = p3[:, :3].astype(np.float64) * scale
    pts[:, 2] += z_offset * scale
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

    ``params`` = (cx, cy, cz, yaw, pitch, roll) in radians / metres.
    Returns (gray_uint8, valid_mask).
    """
    cx, cy, cz, yaw, pitch, roll = params
    rot = R.from_euler('zyx', [yaw, pitch, roll]).as_matrix()
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
    ap.add_argument("--out", default=None, help="write an overlay image here")
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

    # --- stage 1: coarse yaw/pitch on a low-res gradient map (fast) ---
    CW, CH = WRK_W // 3, WRK_H // 3
    cpts, cI = subsample(pts, intensity, OBJ_POINTS)
    ref_edge_c, ref_valid_c = build_reference(ref_gray, CW, CH)
    cost_c = make_cost(cpts, cI, CW, CH, ref_edge_c, ref_valid_c)
    print("coarse search over yaw/pitch ...")
    best = None
    for yaw in np.radians(np.arange(-180, 180, 10)):
        for pitch in np.radians(np.arange(-20, 20, 5)):
            c = cost_c((0.0, 0.0, 0.0, yaw, pitch, 0.0))
            if best is None or c < best[0]:
                best = (c, (0.0, 0.0, 0.0, yaw, pitch, 0.0))
    yaws = np.array([best[1][3]])
    print(f"  init cost {best[0]:.4f}  yaw={np.degrees(best[1][3]):.1f} pitch={np.degrees(best[1][4]):.1f}")

    # --- stage 2: refine all six params at working resolution ---
    fpts, fI = subsample(pts, intensity, OBJ_POINTS)
    ref_edge, ref_valid = build_reference(ref_gray, WRK_W, WRK_H)
    cost = make_cost(fpts, fI, WRK_W, WRK_H, ref_edge, ref_valid)
    print(f"Nelder-Mead refinement ({len(fpts)} pts, {WRK_W}x{WRK_H}) ...")
    res = minimize(cost, best[1], method="Nelder-Mead",
                   options={"maxiter": 3000, "xatol": 1e-4, "fatol": 1e-7})
    cx, cy, cz, yaw, pitch, roll = res.x
    print(f"\ncost {res.fun:.4f}  (yaw array={yaws.tolist()})")
    print(f"  centre (m) : [{cx:.4f}, {cy:.4f}, {cz:.4f}]  "
          f"({cx*1000:.1f}, {cy*1000:.1f}, {cz*1000:.1f} mm)")
    print(f"  yaw   : {np.degrees(yaw):.2f} deg")
    print(f"  pitch : {np.degrees(pitch):.2f} deg")
    print(f"  roll  : {np.degrees(roll):.2f} deg")

    # --- overlay for visual verification ---
    gray, mask = project_pano(fpts, fI, res.x, WRK_W, WRK_H)
    edge = gradient_map(gray, WRK_W, WRK_H)
    overlay = np.zeros((WRK_H, WRK_W, 3), np.uint8)
    overlay[..., 2] = (ref_valid * (ref_edge > 0.15) * 255).astype(np.uint8)   # red = colour edges
    overlay[..., 1] = ((mask.astype(bool) & (edge > 0.15)) * 255).astype(np.uint8)  # green = lidar edges
    out = args.out or os.path.join("/tmp", f"extrinsics_overlay_{args.scan_id}.png")
    cv2.imwrite(out, overlay)
    print(f"\noverlay (red=colour, green=lidar, yellow=match) -> {out}")

    params = {"center_m": [cx, cy, cz], "yaw_deg": float(np.degrees(yaw)),
              "pitch_deg": float(np.degrees(pitch)), "roll_deg": float(np.degrees(roll)),
              "mirror": bool(args.mirror), "cost": float(res.fun)}
    print("\nconfig snippet:\n" + json.dumps({"VERTEXCOLOUR": {"PROJECTION": params}}, indent=2))


if __name__ == "__main__":
    main()
