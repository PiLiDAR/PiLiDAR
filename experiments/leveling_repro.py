"""Reproduce the IMU leveling bias on real scans (throwaway diagnostic).

For each scan pickle, compare three candidate "up" vectors in the IMU body frame:
  - mean gravity      (imu_up_vector: what process_raw currently uses -> biased)
  - reference gravity (gravity at stepper angle ~= 0 -> the proposed fix)
  - turntable axis    (fit_turntable_axis: +Z in the cloud by construction)

The bias we are chasing: over a 180 deg sweep the *mean* gravity lies inside the
gravity half-cone, so leveling by it under-corrects. This prints the tilt of each
candidate w.r.t. the DMP world up so the under-read is visible directly.

Run:  ~/venv/bin/python experiments/leveling_repro.py [scan_id ...]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from lib.pointcloud import load_raw_scan  # noqa: E402
from lib.imu_orientation import (  # noqa: E402
    fit_turntable_axis, imu_up_vector, reference_gravity,
)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _tilt_deg(body_vec, quats, scale=16384.0):
    """Angle between a body-frame guess and the true gravity, averaged over the
    sweep. Reconstructs gravity in the body frame per sample and takes the
    (sign-aligned) mean angle."""
    q = np.asarray(quats, dtype=float) / scale
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    rm = R.from_quat(q[:, [1, 2, 3, 0]]).as_matrix()
    g_body = rm[:, 2, :]                                   # (N,3)
    v = np.asarray(body_vec, dtype=float)
    v = v / np.linalg.norm(v)
    # how far the candidate is from the *mean* gravity direction (what matters
    # for a static up); and its cone-mean angle
    g_mean = g_body.mean(axis=0)
    g_mean /= np.linalg.norm(g_mean)
    return np.degrees(np.arccos(np.clip(abs(np.dot(v, g_mean)), -1, 1)))


def probe(scan_id):
    path = os.path.join(ROOT, "scans", scan_id, f"{scan_id}_lidar.pkl")
    if not os.path.exists(path):
        print(f"[skip] {path} not found")
        return
    raw = load_raw_scan(path)
    quats = raw.get("quaternions")
    z_angles = raw.get("z_angles")
    if quats is None:
        print(f"[{scan_id}] no quaternions stored")
        return
    quats = np.asarray(quats, dtype=float)
    print(f"\n=== {scan_id} : {len(quats)} quats, "
          f"{len(z_angles) if z_angles is not None else 0} z_angles")
    norms = np.linalg.norm(quats, axis=1)
    print(f"    quat norm: mean={norms.mean():.0f} min={norms.min():.0f} "
          f"max={norms.max():.0f} (healthy ~16384)")

    fit = fit_turntable_axis(quats)
    print(f"    fit_turntable_axis: tilt={fit.tilt_deg:.2f} deg  "
          f"spread={fit.spread_deg:.2f} deg  step={fit.step_deg:.2f} deg  n={fit.n}")
    print(f"      c_body       = {np.round(fit.c_body, 4).tolist()}")
    print(f"      c_body_accel = {np.round(fit.c_body_accel, 4).tolist()}")
    print(f"      |c_body . c_body_accel| = "
          f"{abs(np.dot(fit.c_body, fit.c_body_accel)):.4f}")

    up_mean = imu_up_vector(quats)
    up_ref = reference_gravity(quats, z_angles=z_angles)
    for name, vec in (("mean gravity", up_mean),
                      ("reference gravity", up_ref),
                      ("turntable axis", fit.c_body)):
        print(f"    {name:18s} = {np.round(vec, 4).tolist()}  "
              f"|tilt vs mean={_tilt_deg(vec, quats):.2f} deg")
    print(f"    angle(mean gravity, reference gravity) = "
          f"{np.degrees(np.arccos(np.clip(abs(np.dot(up_mean, up_ref)), -1, 1))):.2f} deg")


def main():
    ids = sys.argv[1:] or ["261011-1953", "261011-2031", "261011-2156", "261011-2213"]
    for scan_id in ids:
        probe(scan_id)


if __name__ == "__main__":
    main()