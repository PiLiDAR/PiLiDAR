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
