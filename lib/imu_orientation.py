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
