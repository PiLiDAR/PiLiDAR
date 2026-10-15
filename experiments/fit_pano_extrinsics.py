"""Solve the LiDAR<->camera panorama rotation by mutual information.

The RGB panorama and the LiDAR intensity panorama image the **same scene from the
same nodal point**, so the mapping between them is (to first order) a pure
**rotation of the sphere** -- i.e. the relative orientation of the LiDAR and the
camera. Raw-intensity matching (SIFT/NCC) fails across the two modalities, so this
uses **mutual information**, the standard cross-modal metric: it only requires a
statistical dependence, not a linear intensity relationship.

Search = coarse->fine over (yaw, pitch, roll), each candidate warping the LiDAR
panorama on the sphere and scored by MI against the RGB panorama. The resulting
rotation is exactly what `VERTEXCOLOUR.ROTATE_DEG` implements; with a perfect
mount and no parallax it should be identity (and `CENTER_MM` zero), so this doubles
as an independent check of `3D.CALIBRATION`.

    ~/venv/bin/python experiments/fit_pano_extrinsics.py 261015-1839
"""
import argparse
import itertools
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_gray(path, W, H):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"cannot read {path}")
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


def warp_equirect(img, rot, W, H):
    """Rotate an equirect panorama by ``rot`` (scipy Rotation) -> remap tables."""
    xs = (np.arange(W) + 0.5) / W * 2 * np.pi - np.pi
    ys = np.pi / 2 - (np.arange(H) + 0.5) / H * np.pi
    lon, lat = np.meshgrid(xs, ys)
    d = np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], -1)
    d = d @ rot.as_matrix().T                      # rotate the sample direction
    lon2 = np.arctan2(d[..., 1], d[..., 0])
    lat2 = np.arcsin(np.clip(d[..., 2], -1, 1))
    mx = ((lon2 + np.pi) / (2 * np.pi) * W).astype(np.float32)
    my = ((np.pi / 2 - lat2) / np.pi * H).astype(np.float32)
    return cv2.remap(img, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def mutual_info(a, b, bins=32):
    """Mutual information (bits) between two uint8 images (0 = invalid)."""
    m = (a > 0) & (b > 0)
    if m.sum() < 200:
        return -1.0
    aa = np.clip(a[m].astype(np.int32) * bins // 256, 0, bins - 1)
    bb = np.clip(b[m].astype(np.int32) * bins // 256, 0, bins - 1)
    pab = np.bincount(aa * bins + bb, minlength=bins * bins).astype(np.float64)
    pab /= pab.sum()
    pa = pab.reshape(bins, bins).sum(1)
    pb = pab.reshape(bins, bins).sum(0)
    nz = pab > 0
    return float(np.sum(pab[nz] * np.log2(pab[nz] / (pa.repeat(bins)[nz] * np.tile(pb, bins)[nz]))))


def search(rgb, lidar, base, ranges, step, W, H):
    """Grid search MI over yaw/pitch/roll around ``base`` (deg)."""
    best = (-1.0, np.asarray(base, float))
    for dy, dp, dr in itertools.product(*[np.arange(-r, r + 1e-9, step) for r in ranges]):
        cand = np.asarray(base, float) + [dy, dp, dr]
        warped = warp_equirect(lidar, R.from_euler("xyz", cand, degrees=True), W, H)
        mi = mutual_info(rgb, warped)
        if mi > best[0]:
            best = (mi, cand)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_id")
    ap.add_argument("--rgb", default=None)
    ap.add_argument("--lidar", default=None)
    args = ap.parse_args()

    scan = os.path.join(ROOT, "scans", args.scan_id)
    rgb_path = args.rgb or os.path.join(scan, f"{args.scan_id}_blended_fused.jpg")
    lidar_path = args.lidar or os.path.join(scan, f"{args.scan_id}_lidar.jpg")

    params = (0.0, 0.0, 0.0)
    for (W, H), ranges, step in (((512, 256), (10, 10, 5), 1.0),
                                 ((1024, 512), (2, 2, 2), 0.25),
                                 ((2048, 1024), (0.5, 0.5, 0.5), 0.1)):
        rgb = load_gray(rgb_path, W, H)
        lidar = load_gray(lidar_path, W, H)
        mi, params = search(rgb, lidar, params, ranges, step, W, H)
        print(f"{W}x{H}: MI={mi:.4f} bits  yaw/pitch/roll = "
              f"({params[0]:+.2f}, {params[1]:+.2f}, {params[2]:+.2f}) deg")

    print(f"\nLiDAR -> RGB sphere rotation (deg, euler xyz): "
          f"({params[0]:+.3f}, {params[1]:+.3f}, {params[2]:+.3f})")
    rm = R.from_euler("xyz", params, degrees=True).as_matrix()
    print(f"  total angle = {np.degrees(np.arccos(np.clip((np.trace(rm)-1)/2,-1,1))):.2f} deg")
    print("\nconfig (to align the cloud/colouring to the RGB pano):")
    print('  "VERTEXCOLOUR": { "ROTATE_DEG": '
          f"[{-params[0]:.3f}, {-params[1]:.3f}, {-params[2]:.3f}] }}")

    W, H = 2048, 1024
    rgb = load_gray(rgb_path, W, H)
    lidar = load_gray(lidar_path, W, H)
    warped = warp_equirect(lidar, R.from_euler("xyz", params, degrees=True), W, H)
    e_r = cv2.Canny(cv2.GaussianBlur(rgb, (0, 0), 1.2), 40, 120) > 0
    e_l = cv2.Canny(cv2.GaussianBlur(warped, (0, 0), 1.2), 40, 120) > 0
    ov = np.zeros((H, W, 3), np.uint8)
    ov[..., 2] = e_r * 255
    ov[..., 1] = e_l * 255
    out = os.path.join(scan, "tmp", f"{args.scan_id}_pano_mi_overlay.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, ov)
    stack = np.vstack([cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR), np.full((4, W, 3), 255, np.uint8),
                       cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)])
    cv2.imwrite(os.path.join(scan, "tmp", f"{args.scan_id}_pano_mi_stack.png"), stack)
    print(f"\noverlay (red=RGB edges, green=LiDAR edges) -> {out}")


if __name__ == "__main__":
    main()
