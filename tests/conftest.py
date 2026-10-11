"""Shared fixtures for the PiLiDAR test suite."""
import os

import numpy as np
import pytest

from tests.mock_hardware import CRC_TABLE, build_packet

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture
def crc_table():
    return list(CRC_TABLE)


@pytest.fixture
def lidar():
    """A Lidar instance with parsing state initialised but no hardware opened.

    Bypasses ``Lidar.__init__`` (which opens a real serial port) and sets only
    the attributes the read/decode path needs.
    """
    from lib.lidar_driver import Lidar

    lidar = object.__new__(Lidar)
    lidar.verbose = False
    lidar.start_byte = bytes([0x54])
    lidar.dlength_byte = bytes([0x2C])
    lidar.dlength = 12
    lidar.package_len = 47
    lidar.deg2rad = np.pi / 180
    lidar.offset = 0.0
    lidar.crc_table = list(CRC_TABLE)
    lidar.byte_array = bytearray()
    lidar.dropped_packets = 0
    lidar.malformed_packets = 0
    lidar.dtype = np.float32
    lidar.out_i = 0
    lidar.out_len = 10
    # step-phase anchor: None -> blind packet-frame boundaries (legacy behaviour)
    lidar.step_at_angle = None
    lidar._prev_angle = None
    lidar._crossing = False
    lidar.speeds = np.empty(lidar.out_len, dtype=lidar.dtype)
    lidar.timestamps = np.empty(lidar.out_len, dtype=lidar.dtype)
    lidar.points_2d = np.empty((lidar.out_len * lidar.dlength, 3), dtype=lidar.dtype)
    lidar.angle_package = np.zeros(lidar.dlength)
    lidar.distance_package = np.zeros(lidar.dlength)
    lidar.luminance_package = np.zeros(lidar.dlength)
    lidar.speed = 0
    lidar.timestamp = 0
    lidar.visualization = None
    lidar.z_angles = []
    lidar.cartesian_list = []
    return lidar


def sample_packet(**overrides):
    """A single valid packet with sensible defaults."""
    defaults = dict(
        speed_rps=10.0,
        fsa_deg=0.0,
        lsa_deg=3.0,
        timestamp_ms=1000,
        distances=[1000 + i * 10 for i in range(12)],
        luminances=[128] * 12,
    )
    defaults.update(overrides)
    return build_packet(**defaults)
