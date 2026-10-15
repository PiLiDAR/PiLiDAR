"""Split each LiDAR fan into its two in-plane radial halves and re-align them.

The Waveshare STL27L is a horizontal-plane 2D LiDAR mounted tipped 90 deg to its
side, so its rotation plane becomes the cloud's vertical X-Z plane. Each fan is a
full 360 deg circle in that plane and splits by the **in-plane radial coordinate**
into two halves: ``x < 0`` ("left") and ``x >= 0`` ("right"). Each half therefore
also sweeps a complete 360 deg and sees the **same surfaces** — so, with a correct
assembly, the two halves must **coincide**.

In practice they do not: there is a stable ~2.4 deg relative yaw between the two
halves (see ``docs/lidar_geometry.md``). Assembling the full fan the normal way
(``merge_2D_points`` on the whole raw scan) mixes both halves and produces the
"double walls" artifact.

This module is the production home of the ``experiments/split_fans.py`` logic:

* :func:`split_fans` — split a raw scan into (right, left) half scans.
* :func:`assemble_aligned` — assemble the two halves and apply the **stored**
  left/right offset so they coincide; this is what ``process_raw`` calls.
* :func:`measure_offset` — run ICP between the two halves **once** to estimate
  their relative rigid transform and store it in ``config.json`` (``FAN_SPLIT``).

The stored offset is a rigid transform that maps the **right** half onto the
**left** half, kept in ``config.json`` as ``FAN_SPLIT.TRANSLATION`` (metres, the
frame ICP runs in) + ``FAN_SPLIT.ROTATE_DEG`` (euler xyz, degrees) — the same
convention ``experiments/registration_test.py`` reports. It is a property of the
instrument, so it is measured once and reused for every later scan.
"""

import json
import os

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

try:
    from lib.pointcloud import merge_2D_points, load_raw_scan, lidar_mount
except Exception:
    from pointcloud import merge_2D_points, load_raw_scan, lidar_mount


# ---------------------------------------------------------------------------
# splitting
# ---------------------------------------------------------------------------

def split_fans(raw, axis="x"):
    """Split each 2D fan into (positive, negative) halves by an in-plane axis.

    Fan columns are ``(x, y, intensity)`` with ``x = -r*cos(theta)`` (in-plane
    radial) and ``y = r*sin(theta)`` (in-plane vertical). Returns two
    ``raw_scan``-shaped dicts: ``(pos_raw, neg_raw)`` where ``pos`` is the
    ``x >= 0`` ("right") half and ``neg`` is the ``x < 0`` ("left") half.
    """
    col = {"x": 0, "y": 1}[axis]
    pos_fans, neg_fans = [], []
    for fan in raw["cartesian"]:
        fan = np.asarray(fan)
        if fan.size == 0:
            pos_fans.append(fan)
            neg_fans.append(fan)
            continue
        c = fan[:, col]
        pos_fans.append(fan[c >= 0])
        neg_fans.append(fan[c < 0])
    z = raw["z_angles"]
    return ({"z_angles": z, "cartesian": pos_fans},
            {"z_angles": z, "cartesian": neg_fans})


def _halves(raw, axis="x"):
    """Return ``(left_raw, right_raw)`` — the naming used throughout this module."""
    pos_raw, neg_raw = split_fans(raw, axis)
    return neg_raw, pos_raw


# ---------------------------------------------------------------------------
# offset storage (config.json -> FAN_SPLIT)
# ---------------------------------------------------------------------------

def load_offset(config):
    """The stored right->left 4x4 offset (translation in metres).

    Built from ``FAN_SPLIT.TRANSLATION`` (metres) and ``FAN_SPLIT.ROTATE_DEG``
    (euler xyz, degrees) — the same convention ``registration_test.py`` reports.
    There is deliberately no fallback: both keys must be present in config.json.
    """
    translation = np.asarray(config.get("FAN_SPLIT", "TRANSLATION"), dtype=float)
    rotate_deg = np.asarray(config.get("FAN_SPLIT", "ROTATE_DEG"), dtype=float)
    T = np.eye(4)
    T[:3, :3] = R.from_euler("xyz", rotate_deg, degrees=True).as_matrix()
    T[:3, 3] = translation
    return T


def store_offset(config, T):
    """Persist the right->left offset into ``config.json``.

    Stored as ``FAN_SPLIT.TRANSLATION`` (metres) + ``FAN_SPLIT.ROTATE_DEG``
    (euler xyz, degrees) — the same convention ``registration_test.py`` reports.
    Writes to the **pristine** ``config.json`` on disk (read -> update -> write),
    NOT via ``config.dict``: after ``Config.__init__`` the in-memory dict holds
    non-JSON values (the LiDAR ``start_byte``/``dlength_byte`` are ``bytes`` and
    ``CRC_TABLE`` is a 256-int list), so dumping it would corrupt the file.
    """
    T = np.asarray(T, dtype=float)
    translation = T[:3, 3]
    rotate_deg = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)

    path = os.path.join(config.base_dir, "config.json")
    with open(path, "r") as f:
        data = json.load(f)

    fs = data.setdefault("FAN_SPLIT", {})
    fs["TRANSLATION"] = [float(v) for v in translation]
    fs["ROTATE_DEG"] = [float(v) for v in rotate_deg]
    fs.pop("OFFSET", None)  # new format only: drop the old 4x4 + META
    fs.pop("META", None)

    with open(path, "w") as f:
        json.dump(data, f, indent=4)


# ---------------------------------------------------------------------------
# assembly (used by process_raw)
# ---------------------------------------------------------------------------

def _apply_transform_to_array(arr, T):
    """Apply a 4x4 rigid transform to the xyz of an ``(n, 3|4)`` array.

    Column 3 (laser intensity, if present) is carried through untouched.
    """
    arr = np.asarray(arr, dtype=float)
    xyz = arr[:, :3]
    new_xyz = xyz @ T[:3, :3].T + T[:3, 3]
    if arr.shape[1] > 3:
        return np.column_stack((new_xyz, arr[:, 3:]))
    return new_xyz


def assemble_aligned(config, raw_scan):
    """Assemble the 3D cloud from the two fan halves with the stored offset.

    This is the drop-in replacement for a plain ``merge_2D_points(raw_scan, ...)``
    call in ``process_raw``:

    * ``FAN_SPLIT.ENABLE`` false -> legacy full-fan merge (both halves mixed).
    * enabled -> split into left/right, apply the stored right->left offset to
      the right half, and concatenate the two aligned halves (no more "double
      walls"). With ``ENABLE`` true the offset must be present in config.json
      (``TRANSLATION`` + ``ROTATE_DEG``); a missing offset raises.

    Returns an ``(n_points, 4)`` array in **millimetres** (xyz + intensity), the
    same contract ``merge_2D_points`` returns.
    """
    mount = lidar_mount(config)

    if not config.get("FAN_SPLIT", "ENABLE"):
        return merge_2D_points(raw_scan, mount=mount, up_vector=(0, 0, 1))

    T = load_offset(config)
    axis = config.get("FAN_SPLIT", "AXIS")
    left_raw, right_raw = _halves(raw_scan, axis)

    array_left = merge_2D_points(left_raw, mount=mount, up_vector=(0, 0, 1))
    array_right = merge_2D_points(right_raw, mount=mount, up_vector=(0, 0, 1))

    # the stored offset is in metres (the ICP frame); the assembled clouds are in
    # mm, so scale only the translation by 1000 (the rotation is unitless).
    T_mm = np.eye(4)
    T_mm[:3, :3] = T[:3, :3]
    T_mm[:3, 3] = T[:3, 3] * 1000.0
    array_right = _apply_transform_to_array(array_right, T_mm)

    n_l, n_r = len(array_left), len(array_right)
    print(f"\n[FAN_SPLIT] assembled aligned halves: left {n_l} + right {n_r} "
          f"= {n_l + n_r} points (stored offset applied to the right half).")

    if n_l and n_r:
        return np.vstack([array_left, array_right])
    return array_left if n_l else array_right


# ---------------------------------------------------------------------------
# one-time calibration (ICP between the two halves)
# ---------------------------------------------------------------------------

def _pcd_from_points(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts))
    return pcd


def measure_offset(config, raw_scan=None, save=True, use_global=False, voxel_size=0.15):
    """Estimate the right->left half offset by ICP and (optionally) store it.

    Assembles both halves in **metres** (the frame ICP runs in), downsamples, and
    registers the right half onto the left. By default an identity seed is used —
    the offset is small (~2.4 deg) and stable, so plain ICP converges (fitness
    ~0.99, matching ``experiments/split_fans.py``). Pass ``use_global=True`` to
    seed with feature-based global registration instead.

    Returns the 4x4 right->left transform (translation in metres). With
    ``save=True`` it is written to ``config.json`` as ``FAN_SPLIT.TRANSLATION``
    + ``FAN_SPLIT.ROTATE_DEG``.
    """
    if raw_scan is None:
        raw_scan = load_raw_scan(config.raw_path)

    axis = config.get("FAN_SPLIT", "AXIS")
    left_raw, right_raw = _halves(raw_scan, axis)
    mount = lidar_mount(config)
    scale = config.get("3D", "SCALE")  # mm -> m

    left_pts = merge_2D_points(left_raw, mount=mount, up_vector=(0, 0, 1))[:, :3] * scale
    right_pts = merge_2D_points(right_raw, mount=mount, up_vector=(0, 0, 1))[:, :3] * scale

    left_ds = _pcd_from_points(left_pts).voxel_down_sample(voxel_size)
    right_ds = _pcd_from_points(right_pts).voxel_down_sample(voxel_size)
    print(f"\n[FAN_SPLIT] measuring offset: left {len(left_pts)} / right {len(right_pts)} "
          f"points (downsampled to {len(left_ds.points)} / {len(right_ds.points)})")

    # lazy import: keeps fan_split importable even if the registration stack fails
    from lib.registration import global_registration, icp

    if use_global:
        reg_gr = global_registration(right_ds, left_ds, voxel_size)
        init = reg_gr.transformation
        print(f"[FAN_SPLIT] global registration fitness={reg_gr.fitness:.3f}")
    else:
        init = np.eye(4)

    reg = icp(right_ds, left_ds, init, voxel_size, p2l=True)
    T = reg.transformation  # maps right -> left

    Rm = T[:3, :3]
    angle = np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1)))
    w, v = np.linalg.eig(Rm)
    rot_axis = np.real(v[:, np.argmin(np.abs(w - 1))])
    rot_axis = rot_axis / (np.linalg.norm(rot_axis) + 1e-12)
    t_mm = T[:3, 3] * 1000.0

    print(f"[FAN_SPLIT] right->left ICP: fitness={reg.fitness:.3f} "
          f"rmse={reg.inlier_rmse * 1000:.1f}mm")
    print(f"[FAN_SPLIT]   rotation {angle:.2f} deg about "
          f"({rot_axis[0]:+.2f},{rot_axis[1]:+.2f},{rot_axis[2]:+.2f}); "
          f"translation ({t_mm[0]:+.1f},{t_mm[1]:+.1f},{t_mm[2]:+.1f})mm")

    if save:
        store_offset(config, T)
        print("[FAN_SPLIT] offset stored in config.json "
              "(FAN_SPLIT.TRANSLATION + FAN_SPLIT.ROTATE_DEG).")

    return T


if __name__ == "__main__":
    import argparse
    from lib.config import Config

    ap = argparse.ArgumentParser(description="Measure (and store) the LiDAR "
                         "left/right fan-half offset by ICP.")
    ap.add_argument("scan_id", help="scan id under scans/ (e.g. 261015-1839)")
    ap.add_argument("--no-save", action="store_true",
                    help="measure and print only; do not write config.json")
    ap.add_argument("--global", dest="use_global", action="store_true",
                    help="seed ICP with feature-based global registration")
    ap.add_argument("--voxel", type=float, default=0.15, help="ICP voxel size (m)")
    args = ap.parse_args()

    config = Config()
    config.init(scan_id=args.scan_id)
    measure_offset(config, save=not args.no_save,
                   use_global=args.use_global, voxel_size=args.voxel)
