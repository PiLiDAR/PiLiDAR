"""Generate a minimal synthetic scan under tests/fixtures/.

Run once to (re)create the checked-in fixture:
    python tests/make_fixture.py

Produces a small raw_scan pickle plus a matching lidar/ slice directory so the
point-cloud loading paths can be exercised in CI without real hardware.
"""
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lib.pointcloud import save_raw_scan  # noqa: E402
from tests.mock_hardware import build_packet, make_stream  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, "fixtures")


def _make_slices(n_slices=4, points_per_slice=12):
    """Build a few 2D slices [x, y, intensity] as if from a rotating LiDAR."""
    slices = []
    for s in range(n_slices):
        pts = []
        for i in range(points_per_slice):
            r = 1000.0 + 50 * i
            theta = np.radians(3.0 * i)
            pts.append([-r * np.cos(theta), r * np.sin(theta), 128 + s])
        slices.append(np.array(pts, dtype=np.float32))
    return slices


def main():
    os.makedirs(FIXTURES, exist_ok=True)

    slices = _make_slices()
    z_angles = [0.0, 90.0, 180.0, 270.0]

    raw_scan = {
        "header": {
            "scan_id": "synthetic",
            "device_id": "STL27L",
            "sensor": "synthetic",
            "hardware": "mock",
            "location": "test",
            "author": "pilidar-ci",
        },
        "z_angles": z_angles,
        "angular": None,
        "cartesian": slices,
    }

    pkl_path = os.path.join(FIXTURES, "synthetic_lidar.pkl")
    with open(pkl_path, "wb") as f:
        pickle.dump(raw_scan, f)

    # Open container (NPZ + JSON) — the current on-disk format.
    save_raw_scan(pkl_path, raw_scan)

    # Matching slice directory (plane_<angle>.npy) for the dir-loading path.
    lidar_dir = os.path.join(FIXTURES, "lidar")
    os.makedirs(lidar_dir, exist_ok=True)
    for angle, sl in zip(z_angles, slices):
        np.save(os.path.join(lidar_dir, f"plane_{angle}.npy"), sl)

    # Also drop a raw byte-stream capture for packet-replay tests.
    pkts = [build_packet(10.0, 0.0, 3.0, 1000 * (i + 1),
                         [1000 + i for i in range(12)], [128] * 12)
            for i in range(5)]
    with open(os.path.join(FIXTURES, "capture.bin"), "wb") as f:
        f.write(make_stream(pkts))

    print("Wrote:", pkl_path)
    print("Wrote:", lidar_dir)
    print("Wrote:", os.path.join(FIXTURES, "capture.bin"))


if __name__ == "__main__":
    main()
