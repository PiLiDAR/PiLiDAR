import os

import pytest

from lib.config import Config
from lib.lidar_driver import Lidar


class FakeSerial:
    def __init__(self, data: bytes):
        self._data = data
        self._pointer = 0
        self.is_open = True

    def read(self, size: int = 1) -> bytes:
        if self._pointer >= len(self._data):
            self.is_open = False
            return b""
        chunk = self._data[self._pointer : self._pointer + size]
        self._pointer += size
        if self._pointer >= len(self._data):
            self.is_open = False
        return bytes(chunk)

    def write(self, _):
        return 1

    def close(self):
        self.is_open = False


def build_sample_package(lidar: Lidar) -> bytes:
    data = bytearray(47)
    data[0] = 0x54
    data[1] = 0x2C

    speed = int(360).to_bytes(2, "little")
    data[2:4] = speed

    fsa = int(0).to_bytes(2, "little")
    data[4:6] = fsa

    distance = int(1000).to_bytes(2, "little")
    intensity = 120
    for idx in range(12):
        base = 6 + idx * 3
        data[base : base + 2] = distance
        data[base + 2] = intensity

    lsa = int(1000).to_bytes(2, "little")
    data[42:44] = lsa

    timestamp = int(1234).to_bytes(2, "little")
    data[44:46] = timestamp

    crc = 0
    for byte in data[:-1]:
        crc = lidar.crc_table[(crc ^ byte) & 0xFF]
    data[46] = crc
    return bytes(data)


def test_lidar_collects_full_package(tmp_path):
    config = Config(scans_root=str(tmp_path), use_hardware=False)
    config.set(1, "LIDAR", config.DEVICE, "OUT_LEN")
    config.init(scan_id="test")

    lidar = Lidar(config, serial_connection=FakeSerial(b""))
    package = build_sample_package(lidar)
    lidar.serial_connection = FakeSerial(package)

    lidar.read_loop(callback=lambda: None, max_packages=1)

    assert len(lidar.package_history) == 1
    assert lidar.package_history[0].angles_rad.size == 12

    raw_scan = lidar.create_raw_scan()
    assert os.path.exists(config.raw_path)
    assert raw_scan["packages"][0]["distances_mm"].size == 12

    lidar.close()
