"""Tests for the IMU orientation / leveling helpers (``lib/imu_orientation.py``).

All tests here are **synthetic**: they feed quaternions generated from a known
turntable geometry (see ``_synthesise``) and check that the maths recovers what
was put in — the axis (body + world frames), the rig tilt, the gravity up-vector,
the mounting composition and the leveling rotation.

They verify **self-consistency of the geometry**, *not* real-world accuracy. The
known outstanding problem — leveling a real tilted scan only removes ~60 % of the
tilt — lives in the data, not the maths here, and is documented in
``docs/imu_leveling.md`` (measured against real scans ``261011-1953`` /
``261011-2031``).
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


def _synthesise_sweep(axis_world, c_body, n=400, a0_deg=0.0, a1_deg=180.0):
    """Like ``_synthesise`` but also returns the stepper angles + mounting.

    Returns ``(wxyz_int16, z_angles_deg, mount)`` where ``mount`` is the
    world_from_body rotation at a = 0 deg (so gravity in the body frame at the
    reference attitude is ``mount.inv().apply([0,0,1])``).
    """
    mount = R.align_vectors([axis_world], [c_body])[0]
    a_deg = np.linspace(a0_deg, a1_deg, n)
    qs = np.array([(R.from_rotvec(axis_world * np.radians(a)) * mount).as_quat() for a in a_deg])
    wxyz = qs[:, [3, 0, 1, 2]] * 16384.0
    return wxyz, a_deg, mount


def _angle_deg(u, v):
    u = np.asarray(u, float) / np.linalg.norm(u)
    v = np.asarray(v, float) / np.linalg.norm(v)
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(u, v)), -1, 1))))


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


def test_cloud_up_vector_is_mounting_times_imu_up():
    # cloud_up_vector composes the (explicit) mounting with the measured gravity.
    from lib.imu_orientation import cloud_up_vector, imu_up_vector
    c_body = np.array([0.0, 1.0, 0.0])
    axis_world = np.array([0.0, 0.0, 1.0])
    quats = _synthesise(axis_world, c_body, n=10, tilt_from=0.0, tilt_to=0.0)

    mounting = np.array([[0.0, 0.0, 1.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]])       # arbitrary proper rotation
    up = cloud_up_vector(quats, mounting=mounting)
    assert np.allclose(up, mounting @ imu_up_vector(quats), atol=1e-9)
    assert np.isclose(np.linalg.norm(up), 1.0)


# ---------------------------------------------------------------------------
# reference_gravity: the leveling fix
#
# Over a 180 deg sweep the gravity direction traces only a HALF-cone about the
# turntable axis in the body frame, so the normalized MEAN (imu_up_vector) lies
# inside the cone and under-reports the tilt. reference_gravity samples gravity
# at a single attitude (stepper angle ~= 0 deg) and is not cone-biased.
# ---------------------------------------------------------------------------

def test_mean_gravity_under_reports_tilt_but_reference_recovers_it():
    from lib.imu_orientation import imu_up_vector, reference_gravity

    c_body = np.array([1.0, 0.0, 0.0])
    tilt = 10.0
    axis_world = np.array([0.0, np.sin(np.radians(tilt)), np.cos(np.radians(tilt))])
    wxyz, z_deg, mount = _synthesise_sweep(axis_world, c_body)
    g_true = mount.inv().apply([0.0, 0.0, 1.0])          # gravity in body at a = 0

    mean_err = _angle_deg(imu_up_vector(wxyz), g_true)
    ref_err = _angle_deg(reference_gravity(wxyz, z_angles=z_deg), g_true)

    assert mean_err > 1.1 * tilt          # mean is badly biased (cone half-angle)
    assert ref_err < 0.5                  # reference gravity is essentially exact


def test_reference_gravity_levels_a_tilted_floor():
    from lib.imu_orientation import reference_gravity

    tilt = 12.0
    c_body = np.array([0.3, -0.5, 0.81])
    c_body /= np.linalg.norm(c_body)
    axis_world = np.array([0.0, np.sin(np.radians(tilt)), np.cos(np.radians(tilt))])
    wxyz, z_deg, mount = _synthesise_sweep(axis_world, c_body)
    g_true = mount.inv().apply([0.0, 0.0, 1.0])

    # "level" the cloud by the reference gravity and check the floor is horizontal
    leveled_floor = leveling_rotation(reference_gravity(wxyz, z_angles=z_deg)).apply(g_true)
    assert _angle_deg(leveled_floor, [0.0, 0.0, 1.0]) < 0.5


def test_reference_gravity_falls_back_to_first_sample_without_angles():
    from lib.imu_orientation import reference_gravity

    c_body = np.array([0.0, 1.0, 0.0])
    axis_world = np.array([0.0, np.sin(np.radians(8.0)), np.cos(np.radians(8.0))])
    wxyz, _z_deg, mount = _synthesise_sweep(axis_world, c_body)
    g_true = mount.inv().apply([0.0, 0.0, 1.0])

    # no z_angles -> first attitude (alpha ~ 0) is a fine reference too
    assert _angle_deg(reference_gravity(wxyz), g_true) < 0.5


def test_reference_gravity_drops_extra_leading_warmup_quaternion():
    from lib.imu_orientation import reference_gravity

    c_body = np.array([1.0, 0.0, 0.0])
    axis_world = np.array([0.0, np.sin(np.radians(9.0)), np.cos(np.radians(9.0))])
    wxyz, z_deg, mount = _synthesise_sweep(axis_world, c_body)
    g_true = mount.inv().apply([0.0, 0.0, 1.0])

    # the sweep records one quaternion for the (dropped) warm-up plane, at a stale
    # attitude -> quats leads z_angles by one; reference_gravity must drop it
    stale = np.array([[16384.0, 0.0, 0.0, 0.0]])
    wxyz_ext = np.vstack([stale, wxyz])
    assert len(wxyz_ext) == len(z_deg) + 1

    assert _angle_deg(reference_gravity(wxyz_ext, z_angles=z_deg), g_true) < 0.5


def test_solve_mounting_maps_axis_and_gravity():
    from lib.imu_orientation import solve_mounting

    # body frame: turntable axis +Y, gravity 12 deg off it
    tilt = 12.0
    axis_body = np.array([0.0, 1.0, 0.0])
    gravity_body = np.array([0.0, np.cos(np.radians(tilt)), np.sin(np.radians(tilt))])
    # cloud frame: axis -> +Z, floor normal in the Y-Z plane
    n_cloud = np.array([0.0, np.sin(np.radians(tilt)), np.cos(np.radians(tilt))])

    M = solve_mounting(axis_body, gravity_body, gravity_cloud=n_cloud)
    assert np.allclose(M @ axis_body, [0, 0, 1], atol=1e-6)
    assert np.allclose(M @ gravity_body, n_cloud, atol=1e-6)
    assert np.isclose(np.linalg.det(M), 1.0)                 # proper rotation
    assert np.allclose(M @ M.T, np.eye(3), atol=1e-9)


def test_solve_mounting_default_floor_normal_is_in_yz_plane():
    from lib.imu_orientation import solve_mounting

    tilt = 10.0
    axis_body = np.array([1.0, 0.0, 0.0])
    gravity_body = np.array([np.cos(np.radians(tilt)), np.sin(np.radians(tilt)), 0.0])

    M = solve_mounting(axis_body, gravity_body)
    expected = np.array([0.0, np.sin(np.radians(tilt)), np.cos(np.radians(tilt))])
    assert np.allclose(M @ axis_body, [0, 0, 1], atol=1e-6)
    assert np.allclose(M @ gravity_body, expected, atol=1e-6)
