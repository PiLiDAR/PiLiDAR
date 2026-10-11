"""CRC8 validation and corrupted-packet recovery tests."""
from tests.mock_hardware import CRC_TABLE, build_packet, crc8


def test_crc8_empty_is_zero():
    # CRC8 of an empty buffer is 0 by construction of the table-driven loop.
    assert crc8(b"") == 0


def test_crc8_matches_manual_table_walk():
    data = b"\x54\x2c\x10\x0e"
    expected = 0
    for byte in data:
        expected = CRC_TABLE[(expected ^ byte) & 0xFF]
    assert crc8(data) == expected


def test_build_packet_crc_is_valid(lidar):
    pkt = build_packet(10.0, 0.0, 3.0, 1000, [1000] * 12, [128] * 12)
    assert lidar.check_CRC8(pkt) is True


def test_corrupted_payload_fails_crc(lidar):
    pkt = bytearray(build_packet(10.0, 0.0, 3.0, 1000, [1000] * 12, [128] * 12))
    pkt[10] ^= 0xFF  # flip a distance byte
    assert lidar.check_CRC8(bytes(pkt)) is False


def test_corrupted_crc_byte_fails(lidar):
    pkt = bytearray(build_packet(10.0, 0.0, 3.0, 1000, [1000] * 12, [128] * 12))
    pkt[46] ^= 0x01  # flip the checksum itself
    assert lidar.check_CRC8(bytes(pkt)) is False


def test_check_crc8_with_explicit_crc(lidar):
    data = b"\x54\x2c\x10\x0e\x04\x01"
    good = crc8(data)
    assert lidar.check_CRC8(data, crc=good) is True
    assert lidar.check_CRC8(data, crc=(good + 1) & 0xFF) is False
