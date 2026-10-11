"""Packet decode and polar-to-Cartesian transform tests."""
import numpy as np

from tests.mock_hardware import build_packet, make_stream, MockLidarSerial
from tests.conftest import sample_packet


def test_decode_fields(lidar):
    pkt = build_packet(
        speed_rps=10.0, fsa_deg=0.0, lsa_deg=3.0, timestamp_ms=1234,
        distances=[1000] * 12, luminances=[200] * 12,
    )
    lidar.decode(pkt)
    assert lidar.speed == 10.0
    assert lidar.timestamp == 1234
    assert np.allclose(lidar.distance_package, 1000)
    assert np.all(lidar.luminance_package == 200)
    # angles span FSA..LSA inclusive across 12 samples
    assert np.isclose(lidar.angle_package[0], 0.0 * np.pi / 180)
    assert np.isclose(lidar.angle_package[-1], 3.0 * np.pi / 180)


def test_decode_angle_wraparound(lidar):
    # LSA < FSA means the sweep crosses 0 degrees.
    pkt = build_packet(
        speed_rps=10.0, fsa_deg=358.0, lsa_deg=2.0, timestamp_ms=0,
        distances=[500] * 12, luminances=[10] * 12,
    )
    lidar.decode(pkt)
    # first sample at 358 deg, last at 2 deg (wrapped)
    assert np.isclose(lidar.angle_package[0], 358.0 * np.pi / 180)
    assert np.isclose(lidar.angle_package[-1], 2.0 * np.pi / 180)


def test_read_single_packet(lidar):
    lidar.serial_connection = MockLidarSerial(make_stream([sample_packet()]))
    assert lidar.read() is True
    assert lidar.out_i == 0
    # 12 points written into points_2d
    assert np.all(np.isfinite(lidar.points_2d[:12]))


def test_read_rejects_corrupt_packet(lidar):
    pkt = bytearray(sample_packet())
    pkt[20] ^= 0xFF  # corrupt a payload byte -> CRC fails
    lidar.serial_connection = MockLidarSerial(make_stream([bytes(pkt)]))
    assert lidar.read() is False


def test_read_recovers_after_garbage_prefix(lidar):
    # Prepend non-start garbage; the reader must skip to the real start byte.
    good = sample_packet()
    stream = b"\x00\x01\x02" + good
    lidar.serial_connection = MockLidarSerial(stream)
    assert lidar.read() is True


def test_polar2cartesian_known_point():
    from lib.lidar_driver import Lidar
    # angle 0, distance 1000, offset 0 -> x=-1000, y=0
    x, y = Lidar.polar2cartesian([0.0], [1000.0], 0.0)
    assert np.isclose(x[0], -1000.0)
    assert np.isclose(y[0], 0.0)
    # angle 90 deg (pi/2), distance 1000 -> x~0, y=1000
    x, y = Lidar.polar2cartesian([np.pi / 2], [1000.0], 0.0)
    assert np.isclose(x[0], 0.0, atol=1e-6)
    assert np.isclose(y[0], 1000.0)


def test_polar2cartesian_offset():
    from lib.lidar_driver import Lidar
    # offset shifts the angle; distance magnitude is preserved
    x, y = Lidar.polar2cartesian([0.0], [1000.0], np.pi / 2)
    assert np.isclose(np.hypot(x[0], y[0]), 1000.0)


def test_decode_detects_anchor_crossing_once(lidar):
    lidar.step_at_angle = 50.0
    lidar._prev_angle = None
    lidar.decode(sample_packet(fsa_deg=40.0, lsa_deg=43.0))  # first frame: no previous
    assert lidar._crossing is False
    lidar.decode(sample_packet(fsa_deg=44.0, lsa_deg=47.0))  # arc 40->44, anchor not in it
    assert lidar._crossing is False
    lidar.decode(sample_packet(fsa_deg=52.0, lsa_deg=55.0))  # arc 44->52 contains 50
    assert lidar._crossing is True


def test_decode_ignores_reverse_jump(lidar):
    lidar.step_at_angle = 50.0
    lidar._prev_angle = 100.0
    # a big reverse arc (>180) must not register as a forward crossing
    lidar.decode(sample_packet(fsa_deg=10.0, lsa_deg=13.0))
    assert lidar._crossing is False


def _set_angle(lidar, degrees):
    lidar.angle_package = np.full(lidar.dlength, np.radians(degrees))


def test_plane_complete_when_frame_full(lidar):
    lidar.step_at_angle = None
    lidar.out_i = lidar.out_len
    assert lidar._plane_complete() is True
    lidar.out_i = lidar.out_len - 1
    assert lidar._plane_complete() is False


def test_plane_complete_on_anchor_crossing(lidar):
    lidar.step_at_angle = 50.0
    lidar.out_i = 2                          # far from a full frame (out_len=10)
    lidar._crossing = True
    assert lidar._plane_complete() is True


def test_plane_complete_not_until_crossing(lidar):
    lidar.step_at_angle = 50.0
    lidar.out_i = 2
    lidar._crossing = False                  # anchor not crossed yet
    assert lidar._plane_complete() is False


def test_finalize_plane_labels_captured_angle_and_clears_arm(lidar):
    lidar.z_angle = 10.0
    lidar.out_i = 3
    lidar._crossing = True
    lidar._finalize_plane()
    assert lidar.z_angles == [10.0]          # captured angle, not the post-move one
    assert lidar.cartesian_list[0].shape == (3 * lidar.dlength, 3)
    assert lidar._crossing is False


def test_finalize_plane_flushes_partial_plane(lidar):
    lidar.z_angle = 0.0
    lidar.out_i = 5                          # partial plane still gets flushed
    lidar._finalize_plane()
    assert lidar.cartesian_list[0].shape == (5 * lidar.dlength, 3)
