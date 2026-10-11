"""Hardware mocking & packet-replay harness for desktop / CI testing.

Lets the LiDAR driver run without physical hardware by replaying a synthetic
(or recorded) byte stream through a ``serial.Serial``-compatible interface.

Packet layout (47 bytes, LD06 / STL27L):
    [0]      start_byte   0x54
    [1]      dlength_byte 0x2c
    [2:4]    speed        uint16 LE, /360 -> rps
    [4:6]    FSA          uint16 LE, /100 -> start angle (deg)
    [6:42]   12 samples x 3 bytes:
                 [6+3i : 8+3i]  distance  uint16 LE (mm)
                 [8+3i]         luminance uint8
    [42:44]  LSA          uint16 LE, /100 -> end angle (deg)
    [44:46]  timestamp    uint16 LE (ms)
    [46]     CRC8         over bytes [0:46]
"""
import json
import os

# Protocol constants (mirror config.json LIDAR.protocol)
START_BYTE = 0x54
DLENGTH_BYTE = 0x2C
DLENGTH = 12
PACKAGE_LEN = 47

# Load the project's CRC8 table once.
_CRC_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "lib", "CRC.json")
with open(_CRC_PATH, "r") as _f:
    CRC_TABLE = [int(h, 16) for h in json.load(_f)["CRC_TABLE"]]


def crc8(data, table=None):
    """Compute the CRC8 checksum over ``data`` using the project's table."""
    if table is None:
        table = CRC_TABLE
    crc = 0
    for byte in data:
        crc = table[(crc ^ byte) & 0xFF]
    return crc


def build_packet(speed_rps, fsa_deg, lsa_deg, timestamp_ms, distances, luminances):
    """Assemble a valid 47-byte LiDAR packet.

    Parameters
    ----------
    speed_rps : float   rotational frequency in rev/s (stored as *360)
    fsa_deg, lsa_deg : float  start / end angle in degrees (stored as *100)
    timestamp_ms : int
    distances : sequence of 12 ints (mm)
    luminances : sequence of 12 ints (0-255)
    """
    assert len(distances) == DLENGTH and len(luminances) == DLENGTH

    pkt = bytearray(PACKAGE_LEN)
    pkt[0] = START_BYTE
    pkt[1] = DLENGTH_BYTE
    pkt[2:4] = int(round(speed_rps * 360)).to_bytes(2, "little")
    pkt[4:6] = int(round(fsa_deg * 100)).to_bytes(2, "little")

    for i in range(DLENGTH):
        base = 6 + 3 * i
        pkt[base:base + 2] = int(distances[i]).to_bytes(2, "little")
        pkt[base + 2] = int(luminances[i]) & 0xFF

    pkt[42:44] = int(round(lsa_deg * 100)).to_bytes(2, "little")
    pkt[44:46] = int(timestamp_ms).to_bytes(2, "little")
    pkt[46] = crc8(pkt[:46])
    return bytes(pkt)


def make_stream(packets):
    """Concatenate packets into a single replayable byte stream."""
    return b"".join(packets)


class MockLidarSerial:
    """A ``serial.Serial``-compatible reader over an in-memory byte buffer.

    Supports the subset of the pyserial API used by ``Lidar.read()``:
    ``read(n)``, ``is_open``, and ``close()``.
    """

    def __init__(self, data=b""):
        self._data = bytes(data)
        self._pos = 0
        self.is_open = True

    def read(self, size=1):
        if not self.is_open:
            return b""
        chunk = self._data[self._pos:self._pos + size]
        self._pos += len(chunk)
        return chunk

    def write(self, data):
        # No-op: the mock never needs to send commands to the sensor.
        return len(data)

    def close(self):
        self.is_open = False


class MockStepper:
    """A GPS-free stand-in for ``lib.a4988_driver.A4988``.

    Tracks an internal angle so acquisition workflows can be tested on desktop
    without GPIO. Mirrors the subset of the A4988 API used by ``PiLiDAR.py``:
    ``move_to_angle()``, ``move_steps()``, ``get_current_angle()``.
    """

    def __init__(self, step_angle=1.8, microsteps=16, gear_ratio=1.0):
        self.step_angle = step_angle
        self.microsteps = microsteps
        self.gear_ratio = gear_ratio
        self.current_angle = 0.0
        self.total_steps = 0

    def move_steps(self, steps):
        self.total_steps += steps
        deg_per_step = self.step_angle / (self.microsteps * self.gear_ratio)
        self.current_angle = (self.current_angle + steps * deg_per_step) % 360
        return self.current_angle

    def move_to_angle(self, target_angle):
        delta = (target_angle - self.current_angle) % 360
        deg_per_step = self.step_angle / (self.microsteps * self.gear_ratio)
        steps = int(round(delta / deg_per_step))
        return self.move_steps(steps)

    def get_current_angle(self):
        return self.current_angle


class MockCamera:
    """A Picamera2-free stand-in for ``lib.picam2_utils.Camera``.

    Records the HDR brackets it is asked to take so acquisition tests can assert
    on filenames / EV ordering without a camera.
    """

    def __init__(self, exposure_time=0.0001, gain=1.0, awbgains=(1.0, 1.0)):
        self.exposure_time = exposure_time
        self.gain = gain
        self.awbgains = list(awbgains)
        self.captures = []
        self.configured_dims = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def estimate_parameters(self, config):
        return self.exposure_time, self.gain, self.awbgains

    def configure(self, still_dims, lores_dims=None):
        self.configured_dims = tuple(still_dims)

    def take_photo(self, path, exposure_time, gain, awbgains):
        self.captures.append(path)
        return path

    def take_hdr(self, path, AEB, AEB_stops, exposure_time, gain, awbgains):
        import os
        base, ext = os.path.splitext(path)
        paths = [f"{base}_{step * AEB_stops}{ext}" for step in range(-AEB, AEB + 1)]
        self.captures.extend(paths)
        return paths
