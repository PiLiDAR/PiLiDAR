"""Tests for the turntable-axis fit (``lib/imu_orientation.py``).

Synthesises an IMU on a tilted turntable (known axis) and checks the fit
recovers it, in both the body and world frames.
"""
import numpy as np
from scipy.spatial.transform import Rotation as R

from lib.imu_orientation import fit_turntable_axis, imu_up_vector, leveling_rotation


def _synthesise(axis_world, c_body, n=400, tilt_from=0.0, tilt_to=np.pi):
    """Build DMP quaternions for an IMU revolving about ``c_body``.

    ``axis_world`` is the turntable axis in the gravity-referenced world frame.
    The IMU body orientation (world_from_body) is ``R(axis_world, a) @ mount``,
    where ``mount`` maps the body axis onto the world axis. Returns the int16
    wxyz array as the driver would emit it.
    """
    # world_from_body at a=0: maps c_body -> axis_world
    mount = R.align_vectors([axis_world], [c_body])[0]
    qs = []
    angles = np.linspace(tilt_from, tilt_to, n)
    for a in angles:
        world_from_body = R.from_rotvec(axis_world * a) * mount
        qs.append(world_from_body.as_quat())       # xyzw
    qs = np.array(qs)
    wxyz = qs[:, [3, 0, 1, 2]]                     # -> wxyz
    return wxyz * 16384.0


def test_fit_recovers_body_axis():
    c_body = np.array([-0.62, -0.78, 0.07])
    c_body /= np.linalg.norm(c_body)
    axis_world = np.array([0.0, 0.0, 1.0])         # level rig

    fit = fit_turntable_axis(_synthesise(axis_world, c_body))
    assert np.isclose(abs(np.dot(fit.c_body, c_body)), 1.0, atol=1e-2)
    assert fit.n == 400
    assert abs(fit.step_deg - np.degrees(np.pi / 399)) < 0.1


def test_fit_recovers_world_axis_and_tilt():
    c_body = np.array([1.0, 0.0, 0.0])
    axis_world = np.array([0.0, np.sin(np.radians(20)), np.cos(np.radians(20))])

    fit = fit_turntable_axis(_synthesise(axis_world, c_body))
    assert np.isclose(abs(np.dot(fit.w_world, axis_world)), 1.0, atol=1e-2)
    assert abs(fit.tilt_deg - 20.0) < 1.0          # rig tilted 20° off gravity


def test_accel_and_gyro_axis_agree():
    c_body = np.array([0.3, -0.5, 0.81])
    c_body /= np.linalg.norm(c_body)
    # a TILTED rig so gravity sweeps a cone in the body frame (a level rig would
    # keep gravity constant and the plane fit degenerate)
    axis_world = np.array([0.0, np.sin(np.radians(15)), np.cos(np.radians(15))])

    fit = fit_turntable_axis(_synthesise(axis_world, c_body))
    # the gravity estimate should match the incremental-rotation estimate
    assert np.isclose(abs(np.dot(fit.c_body, fit.c_body_accel)), 1.0, atol=5e-2)
    assert fit.spread_deg < 1.0


def test_accel_axis_for_level_rig_is_gravity_direction():
    c_body = np.array([0.2, 0.1, 0.97])
    c_body /= np.linalg.norm(c_body)
    axis_world = np.array([0.0, 0.0, 1.0])   # level rig -> gravity constant in body

    fit = fit_turntable_axis(_synthesise(axis_world, c_body))
    assert np.isclose(abs(np.dot(fit.c_body, fit.c_body_accel)), 1.0, atol=1e-3)


def test_imu_up_vector_recovers_gravity_direction():
    # a static IMU whose body up is tilted: gravity in body should be found
    c_body = np.array([0.1, -0.3, 0.95])
    c_body /= np.linalg.norm(c_body)
    axis_world = np.array([0.0, 0.0, 1.0])
    quats = _synthesise(axis_world, c_body, n=50, tilt_from=0.0, tilt_to=0.0)

    up = imu_up_vector(quats)
    assert np.isclose(abs(np.dot(up, c_body)), 1.0, atol=1e-6)


def test_leveling_rotation_maps_up_to_world_z():
    up = np.array([0.2, -0.1, 0.97])
    up /= np.linalg.norm(up)
    rot = leveling_rotation(up)
    assert np.allclose(rot.apply(up), [0, 0, 1], atol=1e-6)


def test_cloud_up_vector_maps_imu_y_to_cloud_z():
    # scanner level: up = +Y in IMU frame -> cloud up vector should be +Z
    from lib.imu_orientation import cloud_up_vector, _IMU_TO_CLOUD
    # a static IMU whose body up (gravity dir) is +Y
    c_body = np.array([0.0, 1.0, 0.0])
    axis_world = np.array([0.0, 0.0, 1.0])
    quats = _synthesise(axis_world, c_body, n=10, tilt_from=0.0, tilt_to=0.0)
    up = cloud_up_vector(quats)
    assert np.isclose(abs(np.dot(up, [0, 0, 1])), 1.0, atol=1e-6)
    # the mapping is a proper rotation
    assert np.isclose(np.linalg.det(_IMU_TO_CLOUD), 1.0)
