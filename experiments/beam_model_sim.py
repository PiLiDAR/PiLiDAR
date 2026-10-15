"""Raycast a synthetic room with an imperfect (non-planar) spinning 2D-LiDAR model.

The STL27L is a horizontal-plane 2D LiDAR (spinning ToF head). PiLiDAR mounts it
tipped 90 deg, so its **rotation plane maps to the cloud's vertical X-Z plane** and
its **spindle axis maps to cloud Y** (the disc normal). Any out-of-plane defect --
a diode sitting off the plane, or the emitted "plane" not being a perfect plane at
all -- means the beam does not lie in the rotation plane: it traces a small
deviation angle ``delta(theta)`` out of the plane.

Models (``--mode``):
  * ``cone``   -- fixed diode tilt: delta = const        (the beam on a cone)
  * ``tilt``   -- the whole emitted plane tilted: delta ~ sin(theta)   (a warped plane)
  * ``wobble`` -- delta const but offset once per revolution (same as cone here)
  * ``none``   -- a perfect plane (baseline)

The reconstruction is done the NORMAL way (from the reported in-plane angle +
range, discarding the out-of-plane component), then the fan is split into its
left/right radial halves and their relative yaw measured -- globally and per
fan-elevation theta -- so a model's signature can be compared with a real scan.

    ~/venv/bin/python experiments/beam_model_sim.py
    ~/venv/bin/python experiments/beam_model_sim.py --gamma 1.0 --mode tilt
"""
import argparse

import numpy as np
import open3d as o3d

# a small asymmetric room (axis, coordinate); includes ceiling/floor so rays terminate
WALLS = [(0, 4.0), (0, -3.0), (1, 4.0), (1, -5.0), (2, 2.5), (2, -2.0)]


def sim(gamma_deg, mode="cone", nphi=720, nth=360):
    """Raycast the room; return the reconstructed points + per-point fan angle (deg)."""
    g = np.radians(gamma_deg)
    th = np.radians(np.linspace(-180, 180, nth, endpoint=False))
    ph = np.radians(np.linspace(0, 360, nphi, endpoint=False))
    ct, st = np.cos(th), np.sin(th)

    # module beam direction (3, nth); out-of-plane angle delta(theta)
    if mode == "cone" or mode == "wobble":
        dm = np.stack([np.cos(g) * ct, np.cos(g) * st, np.full(nth, np.sin(g))])
    elif mode == "tilt":
        dm = np.stack([ct, st * np.cos(g), st * np.sin(g)])
    elif mode == "none":
        dm = np.stack([ct, st, np.zeros(nth)])
    else:
        raise ValueError(mode)

    dc = np.stack([-dm[0], dm[2], dm[1]])            # module -> cloud (x -> -X, z->Y, y->Z)
    p3 = np.empty((nphi, nth, 3))
    for i, p in enumerate(ph):
        c, s = np.cos(-p), np.sin(-p)
        x = c * dc[0] - s * dc[1]
        y = s * dc[0] + c * dc[1]
        z = dc[2]
        t = np.full(nth, np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            for axis, comp in ((0, x), (1, y), (2, z)):
                for val in {WALLS[j][1] for j in range(len(WALLS)) if WALLS[j][0] == axis}:
                    tt = val / comp
                    t = np.where((tt > 0.05) & (tt < t), tt, t)
        # reconstruct from (range, reported in-plane angle) -- out-of-plane is discarded
        X = -t * ct
        Z = t * st
        p3[i] = np.stack([c * X, s * X, Z], axis=1)
    p3 = p3.reshape(-1, 3)
    good = np.isfinite(p3).all(axis=1) & (np.hypot(p3[:, 0], p3[:, 1]) < 20)
    return p3[good], np.degrees(th).repeat(nphi)[good]


def rel_yaw(a, b, vo=0.03, thr=0.2):
    pa = o3d.geometry.PointCloud(); pa.points = o3d.utility.Vector3dVector(a)
    pb = o3d.geometry.PointCloud(); pb.points = o3d.utility.Vector3dVector(b)
    pa = pa.voxel_down_sample(vo); pb = pb.voxel_down_sample(vo)
    if len(pa.points) < 100 or len(pb.points) < 100:
        return None
    r = o3d.pipelines.registration.registration_icp(
        pb, pa, thr, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    T = r.transformation
    return np.degrees(np.arctan2(T[1, 0], T[0, 0])), r.inlier_rmse * 1000


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gamma", type=float, nargs="+", default=[0.0, 0.5, 1.0, 2.0],
                    help="out-of-plane deviation angle(s) in deg")
    ap.add_argument("--mode", nargs="+", default=["cone", "tilt", "wobble"],
                    choices=["cone", "tilt", "wobble", "none"])
    args = ap.parse_args()
    print("signature = relative left/right yaw (real scan 261015-1726: flat ~-2.4 deg)\n")
    for g in args.gamma:
        for mode in args.mode:
            P, _ = sim(g, mode)
            eta = np.degrees(np.arctan2(P[:, 2], np.hypot(P[:, 0], P[:, 1])))
            L, eL = P[P[:, 0] < 0], eta[P[:, 0] < 0]
            Rt, eR = P[P[:, 0] >= 0], eta[P[:, 0] >= 0]
            res = rel_yaw(L, Rt)
            line = f"gamma={g:.1f} {mode:6s}: rel-yaw={res[0]:+.2f} (rmse {res[1]:.0f}mm) | per-theta:"
            for lo in (-60, -30, 0, 30):
                mL = (eL >= lo) & (eL < lo + 30); mR = (eR >= lo) & (eR < lo + 30)
                r = rel_yaw(L[mL], Rt[mR])
                line += f" {r[0]:+.2f}" if r else " n/a"
            print(line)


if __name__ == "__main__":
    main()