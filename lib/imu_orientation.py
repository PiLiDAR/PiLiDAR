"""
IMU orientation utilities (MPU6050 / DMP quaternions).

Three loosely-coupled parts:

1. **Quaternion helpers** — ``DMP_get_euler`` / ``DMP_get_roll_pitch_yaw`` and
   the :class:`Orientation` wrapper convert the DMP int16 wxyz quaternions
   (unit norm ~16384) into euler / roll-pitch-yaw angles. (Near the 90° pitch
   singularity these are ill-conditioned — prefer the vector methods below.)

2. **Turntable-axis fit** — :func:`fit_turntable_axis` recovers the turntable's
   rotation axis from a sweep's quaternions two independent ways: the common axis
   of the incremental rotations (body + gravity-referenced world frames) and a
   gravity-cone plane fit. This is the basis for the scanner "up".

3. **Leveling** — turns a measured "up" into a rotation that levels the assembled
   point cloud (see ``docs/imu_leveling.md``).

Coordinate frames
-----------------
- **IMU body**: X = left, Y = up, Z = forward (as mounted on the scanner).
- **DMP world**: gravity-referenced, +Z = up (accel/gyro fused; yaw drifts).
- **Cloud/scanner**: +Z = turntable axis; the cloud is assembled in this frame by
  ``lib/pointcloud.py`` and is *level by construction only if the rig is level*.

The IMU sits on the rotating platform, so its quaternions describe a rotation
about the turntable axis. The constant **IMU -> cloud mounting rotation** is
calibration data and lives in ``config.json`` as ``IMU.MOUNTING`` (there is no
in-code default); see :func:`mounting_matrix`.

Units: DMP quaternions are int16 with a unit norm of 16384; functions take a
``scale`` (default 16384) to normalize. Angles are radians unless named ``_deg``.
"""

import open3d as o3d
import numpy as np
try:
    from quat import XYZVector as V
    from quat import Quaternion as Q
except ImportError:
    from collections import namedtuple
    V = namedtuple('XYZVector', ['x', 'y', 'z'])
    Q = namedtuple('Quaternion', ['w', 'x', 'y', 'z'])

from collections import namedtuple

from scipy.spatial.transform import Rotation as R

# result of fit_turntable_axis
AxisFit = namedtuple("AxisFit", [
    "c_body",        # turntable up axis in the IMU body frame (unit)
    "w_world",       # same axis in the DMP world frame (unit, gravity-referenced)
    "tilt_deg",      # angle between w_world and gravity up -> rig inclination
    "c_body_accel",  # body axis from the gravity-cone fit (independent cross-check)
    "spread_deg",    # consistency of the per-step axes (lower = cleaner)
    "step_deg",      # mean per-step rotation angle (should match the stepper)
    "n",             # number of usable samples
])

_WORLD_UP = np.array([0.0, 0.0, 1.0])


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def fit_turntable_axis(quaternions, scale=16384.0):
    """Fit the turntable's rotation axis from a sweep's IMU quaternions.

    The IMU revolves about the stepper axis, so every incremental rotation
    ``q_k^{-1} q_{k+1}`` is about that same axis. This returns the axis in both
    the IMU body frame (``c_body``) and the gravity-referenced DMP world frame
    (``w_world``), plus the rig tilt (``w_world`` vs gravity) — averaging over the
    whole sweep cancels gyro/yaw drift.

    ``quaternions`` are the DMP int16 wxyz samples (as stored in
    ``raw_scan["quaternions"]``). ``c_body_accel`` is an independent estimate from
    fitting a plane to the gravity vectors (drift-free accel only); if it agrees
    with ``c_body`` the fit is trustworthy.
    """
    q = np.asarray(quaternions, dtype=float) / scale          # wxyz, DMP scale
    q /= np.linalg.norm(q, axis=1, keepdims=True)             # normalize
    rm = R.from_quat(q[:, [1, 2, 3, 0]]).as_matrix()          # (N,3,3) body->world

    # --- incremental fit: axis of each successive rotation ---
    body_axes, world_axes, angles = [], [], []
    for k in range(len(rm) - 1):
        d_body = rm[k].T @ rm[k + 1]          # increment expressed in the body frame
        rv_b = R.from_matrix(d_body).as_rotvec()
        ang = np.linalg.norm(rv_b)
        if ang < 1e-6:                        # skip duplicate / zero-angle samples
            continue
        d_world = rm[k + 1] @ rm[k].T
        body_axes.append(rv_b / ang)
        world_axes.append(_unit(R.from_matrix(d_world).as_rotvec()))
        angles.append(ang)

    if not body_axes:
        raise ValueError("not enough motion to fit a turntable axis")

    angles = np.asarray(angles)
    c_body = _weighted_axis(np.asarray(body_axes), angles)
    w_world = _weighted_axis(np.asarray(world_axes), angles)

    # --- gravity-based axis (accel only, drift-free cross-check) ---
    # gravity seen in the body frame = R^T @ z (third row of world_from_body).
    # A tilted rig sweeps it on a cone about the axis -> fit the plane. A level
    # rig (axis == gravity) keeps it constant -> that constant IS the axis.
    g_body = rm[:, 2, :]
    if g_body.std(axis=0).max() < 1e-6:
        c_body_accel = _unit(g_body.mean(axis=0))          # level rig
    else:
        _, _, vt = np.linalg.svd(g_body - g_body.mean(axis=0))
        c_body_accel = vt[2]                               # tilted rig: cone axis

    tilt_deg = np.degrees(np.arccos(np.clip(abs(np.dot(w_world, _WORLD_UP)), -1, 1)))
    spread = np.degrees(np.arccos(np.clip(np.abs(body_axes @ c_body), -1, 1)))

    return AxisFit(
        c_body=c_body,
        w_world=w_world,
        tilt_deg=float(tilt_deg),
        c_body_accel=c_body_accel,
        spread_deg=float(spread.std()),
        step_deg=float(np.degrees(angles.mean())),
        n=len(rm),
    )


def _weighted_axis(axes, weights):
    """Angle-weighted mean of unit axes, sign-aligned (axes are ±ambivalent)."""
    ref = _unit((axes * weights[:, None]).sum(axis=0))
    aligned = np.where((axes @ ref)[:, None] < 0, -axes, axes)
    return _unit((aligned * weights[:, None]).sum(axis=0))


def DMP_get_roll_pitch_yaw(a_quat):
    quat = R.from_quat((a_quat.x,a_quat.y,a_quat.z,a_quat.w))
    rpy = quat.as_euler('xyz', degrees=0)
    return V(rpy[0],rpy[1],rpy[2])

def rad2deg(xyz):
    r2d_factor = 180.0 / np.pi
    x_deg = xyz.x * r2d_factor
    y_deg = xyz.y * r2d_factor
    z_deg = xyz.z * r2d_factor
    return V(x_deg, y_deg, z_deg)

def DMP_get_euler(a_quat):
    psi = np.arctan2(2 * a_quat.x * a_quat.y - 2 * a_quat.w * a_quat.z, 2 * a_quat.w * a_quat.w + 2 * a_quat.x * a_quat.x - 1)
    theta = -np.arcsin(2 * a_quat.x * a_quat.z + 2 * a_quat.w * a_quat.y)
    phi = np.arctan2(2 * a_quat.y * a_quat.z - 2 * a_quat.w * a_quat.x, 2 * a_quat.w * a_quat.w + 2 * a_quat.z * a_quat.z - 1)
    return V(psi, theta, phi)

class Orientation:
    def __init__(self, q_list_int16, degrees=False):
        self.quat_list  = []
        self.euler_list = []
        self.rpy_list   = []

        for q_values in q_list_int16:
            quat = self.create_quaternion(q_values / 16384.0)
            self.quat_list.append(quat)

        for quat in self.quat_list:
            self.euler_list.append(self.get_euler(quat, degrees=degrees))
            self.rpy_list.append(self.get_rpy(quat, degrees=degrees))

    def create_quaternion(self, q_values):
        # Convert to quaternion object
        return Q(q_values[0], q_values[1], q_values[2], q_values[3])

    @staticmethod
    def get_euler(quat, degrees=False):
        # Convert to Euler angles (psi, theta, phi)
        euler = DMP_get_euler(quat)
        if degrees:
            euler = rad2deg(euler)
        return euler

    @staticmethod
    def get_rpy(quat, degrees=False):
        # Convert to RPY angles
        rpy = DMP_get_roll_pitch_yaw(quat)
        if degrees:
            rpy = rad2deg(rpy)
        return rpy

# ---------------------------------------------------------------------------
# up-vector and leveling
# ---------------------------------------------------------------------------

# gravity in the DMP's gravity-referenced world frame (its Z axis is up)
_WORLD_UP = np.array([0.0, 0.0, 1.0])


def imu_up_vector(quaternions, scale=16384.0):
    """Scanner up-axis (the turntable axis) in the IMU body frame.

    The mean of the per-sample gravity directions (``R^T @ world_up``); drift-free
    and already the axis projection for a level rig. For a strongly tilted rig
    prefer ``fit_turntable_axis(...).c_body`` (the incremental-rotation estimate).
    """
    q = np.asarray(quaternions, dtype=float) / scale
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    g_body = R.from_quat(q[:, [1, 2, 3, 0]]).as_matrix()[:, 2, :]
    return _unit(g_body.mean(axis=0))


def leveling_rotation(up_vector):
    """Rotation that levels the cloud: map ``up_vector`` (the scanner's up, in
    whatever frame the cloud shares) onto the world up ``(0,0,1)``."""
    up = _unit(np.asarray(up_vector, dtype=float))
    return R.align_vectors([_WORLD_UP], [up])[0]


# The IMU -> cloud mounting is calibration data and lives in config
# (`IMU.MOUNTING`), solved from scans (2026-10-11, scans 261011-1953 level +
# 261011-2031 tilted ~11 deg) from two vector correspondences (turntable axis
# c_body -> cloud +Z, gravity g_body -> the floor normal). It maps the IMU body
# frame onto the cloud frame. See docs/imu_leveling.md and tests/test_imu_axis.py.


def mounting_matrix(config):
    """IMU -> cloud mounting rotation (3x3) from ``IMU.MOUNTING`` (config.json).

    There is deliberately no fallback: the mounting is calibration data and must
    be present in the config for leveling to run.
    """
    try:
        return np.asarray(config.get("IMU", "MOUNTING"), dtype=float)
    except KeyError:
        raise KeyError("config IMU.MOUNTING is required for IMU leveling "
                       "(see docs/imu_leveling.md)")


def cloud_up_vector(quaternions, mounting, scale=16384.0):
    """Gravity direction (scanner up) expressed in the cloud/scanner frame.

    ``mounting`` is the IMU->cloud rotation (``mounting_matrix(config)``). Maps
    the measured gravity (``imu_up_vector``) into the cloud frame, ready for
    :func:`leveling_rotation`.
    """
    return np.asarray(mounting, dtype=float) @ imu_up_vector(quaternions, scale=scale)


if __name__ == '__main__':

    visualize = True

    if visualize:
        arrows = []

        def euler_to_rotation_matrix(euler):
            """Convert Euler angles to a rotation matrix."""
            return R.from_euler('xyz', [euler.x, euler.y, euler.z]).as_matrix()

        def create_arrow(length=1.0, radius=0.02, resolution=4, color=[1, 0, 0]):
            """Create an arrow mesh."""
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=radius,
                cone_radius=radius * 1.5,
                cylinder_height=length,
                cone_height=length * 0.2,
                resolution=resolution,
                cylinder_split=4,
                cone_split=1
            )
            arrow.paint_uniform_color(color)
            return arrow


    q_list_int16 = np.load('scans/quaternion_test.npz')['quat']  # TODO: load from raw_data["quaternion"] instead
    orientation = Orientation(q_list_int16, degrees=True)


    for i, euler_angles in enumerate(orientation.euler_list):
        print(f"Euler {i}: {euler_angles.x:.3} {euler_angles.y:.3} {euler_angles.z:.3}")

        if visualize:
            rotation_matrix = euler_to_rotation_matrix(euler_angles)
            arrow = create_arrow()
            arrow.rotate(rotation_matrix, center=(0, 0, 0))
            arrows.append(arrow)

    if visualize:
        # Visualize the arrows using Open3D
        o3d.visualization.draw_geometries(arrows)
