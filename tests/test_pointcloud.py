"""Point cloud I/O and merge tests (no open3d rendering needed)."""
import os
import pickle

import numpy as np

from lib.pointcloud import load_raw_scan, save_raw_scan

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")


def test_load_synthetic_fixture():
    """The checked-in synthetic scan loads and merges into a 3D cloud."""
    from lib.pointcloud import merge_2D_points

    with open(os.path.join(FIXTURES, "synthetic_lidar.pkl"), "rb") as f:
        raw_scan = pickle.load(f)

    assert len(raw_scan["cartesian"]) == 4
    result = merge_2D_points(raw_scan, z_step=90, position_offset=(0, 0, 0), angle_offset=0)
    assert result.shape[1] == 4
    assert result.shape[0] == 48  # 4 slices x 12 points
    assert not np.isnan(result).any()


def test_open_container_roundtrip(tmp_path):
    """save_raw_scan <-> load_raw_scan preserve a raw scan as NPZ + JSON."""
    fans = [np.random.rand(5, 3), np.random.rand(7, 3), np.random.rand(3, 3)]
    raw_scan = {
        "header": {"scan_id": "rt", "device_id": "STL27L"},
        "z_angles": [0.0, 90.0, 180.0],
        "angular": None,
        "cartesian": fans,
        "quaternions": np.arange(12, dtype=np.int16).reshape(3, 4),
    }
    path = str(tmp_path / "rt_lidar.pkl")           # legacy-style path -> .npz/.json
    save_raw_scan(path, raw_scan)

    assert os.path.exists(str(tmp_path / "rt_lidar.npz"))
    assert os.path.exists(str(tmp_path / "rt_lidar.json"))

    # the sidecar schema must describe the arrays that were actually written
    import json

    meta = json.load(open(str(tmp_path / "rt_lidar.json")))
    with np.load(str(tmp_path / "rt_lidar.npz")) as npz:
        for name, arr in npz.items():
            assert meta["arrays"][name].startswith(str(arr.dtype))
            assert str(list(arr.shape)) in meta["arrays"][name]
    assert meta["n_planes"] == 3
    assert meta["n_quaternions"] == 3
    assert meta["warmup_quaternion_extra"] == 0

    loaded = load_raw_scan(path)
    assert loaded["header"]["scan_id"] == "rt"
    assert np.allclose(loaded["z_angles"], [0.0, 90.0, 180.0])
    assert len(loaded["cartesian"]) == 3
    for got, want in zip(loaded["cartesian"], fans):
        assert got.shape == want.shape
        assert np.allclose(got, want)
    assert np.array_equal(loaded["quaternions"], raw_scan["quaternions"])


def test_open_container_without_quaternions(tmp_path):
    raw_scan = {"header": {}, "z_angles": [0.0], "cartesian": [np.random.rand(4, 3)]}
    path = str(tmp_path / "noq_lidar.pkl")
    save_raw_scan(path, raw_scan)
    loaded = load_raw_scan(path)
    assert "quaternions" not in loaded
    assert len(loaded["cartesian"]) == 1


def test_get_scan_dict_accepts_custom_header():
    from lib.pointcloud import get_scan_dict
    prov = {"scan_id": "abc", "software_version": "9.9.9"}
    d = get_scan_dict([0.0, 90.0], cartesian_list=[np.zeros((1, 3))] * 2, header=prov)
    assert d["header"]["scan_id"] == "abc"
    assert d["header"]["software_version"] == "9.9.9"


def test_scan_provenance_best_effort():
    """Provenance fills the fields it can and never raises."""
    from lib.file_utils import scan_provenance

    class FakeConfig:
        scan_id = "261012-0037"
        target_res = 1 / 6
        h_res = 0.1817

        def get(self, *keys):
            table = {
                ("LIDAR", "DEVICE"): "STL27L",
                ("LIDAR", "STL27L", "SAMPLING_RATE"): 21600,
                ("STEPPER", "SCAN_ANGLE"): 180,
                ("IMU", "LEVEL"): True,
            }
            return table[keys]

    prov = scan_provenance(FakeConfig())
    assert prov["scan_id"] == "261012-0037"
    assert prov["device_id"] == "STL27L"
    assert prov["software_version"]           # non-empty
    assert prov["platform"] and prov["hardware"]
    assert prov["config"]["sampling_rate"] == 21600
    assert prov["config"]["imu_leveling"] is True


def test_load_falls_back_to_legacy_pickle(tmp_path):
    """A plain .pkl with no sibling .npz still loads."""
    raw_scan = {"header": {}, "z_angles": [0.0, 90.0],
                "cartesian": [np.random.rand(3, 3), np.random.rand(3, 3)]}
    path = str(tmp_path / "legacy_lidar.pkl")
    with open(path, "wb") as f:
        pickle.dump(raw_scan, f)
    loaded = load_raw_scan(path)
    assert len(loaded["cartesian"]) == 2


def test_load_slice_directory():
    """The lidar/ slice directory loads via numpy."""
    lidar_dir = os.path.join(FIXTURES, "lidar")
    files = sorted(os.listdir(lidar_dir))
    assert len(files) == 4
    for fn in files:
        arr = np.load(os.path.join(lidar_dir, fn))
        assert arr.shape == (12, 3)


def test_replay_capture_stream():
    """The recorded byte stream replays through the mock serial reader."""
    from tests.mock_hardware import MockLidarSerial

    with open(os.path.join(FIXTURES, "capture.bin"), "rb") as f:
        data = f.read()

    mock = MockLidarSerial(data)
    # 5 packets x 47 bytes
    assert len(data) == 5 * 47
    first = mock.read(47)
    assert first[0] == 0x54 and first[1] == 0x2C


def test_merge_2D_points_basic():
    from lib.pointcloud import merge_2D_points

    # Two 2D slices, each 3 points [x, y, intensity]
    slice1 = np.array([[1.0, 2.0, 100], [3.0, 4.0, 150], [5.0, 6.0, 200]], dtype=np.float32)
    slice2 = np.array([[7.0, 8.0, 120], [9.0, 10.0, 180], [11.0, 12.0, 220]], dtype=np.float32)

    raw_scan = {
        "z_angles": [0.0, 90.0],
        "cartesian": [slice1, slice2],
    }

    result = merge_2D_points(raw_scan, z_step=90, position_offset=(0, 0, 0), angle_offset=0)
    # 6 points total, each with 4 columns (x, y, z, intensity)
    assert result.shape == (6, 4)
    # No NaNs
    assert not np.isnan(result).any()


def test_merge_2D_points_with_offset():
    from lib.pointcloud import merge_2D_points

    slice1 = np.array([[1.0, 0.0, 100]], dtype=np.float32)
    raw_scan = {"z_angles": [0.0], "cartesian": [slice1]}

    # Y offset shifts the cloud
    result = merge_2D_points(raw_scan, position_offset=(0, 500.0, 0), angle_offset=0)
    assert result.shape == (1, 4)
    # The Y coordinate should be shifted by 500
    assert np.isclose(result[0, 1], 500.0, atol=1.0)


def test_remove_NaN():
    from lib.pointcloud import remove_NaN

    arr = np.array([[1.0, 2.0], [np.nan, 3.0], [4.0, 5.0]])
    cleaned = remove_NaN(arr)
    assert cleaned.shape == (2, 2)
    assert not np.isnan(cleaned).any()


def test_pcd_from_np_2d():
    """pcd_from_np with 2D points (XYI columns)."""
    from lib.pointcloud import pcd_from_np

    arr = np.array([[1.0, 2.0, 128], [3.0, 4.0, 255]], dtype=np.float64)
    pcd = pcd_from_np(arr, columns="XYI", estimate_normals=False)
    assert len(pcd.points) == 2
    assert len(pcd.colors) == 2


def test_pcd_from_np_3d():
    """pcd_from_np with 3D points (XYZI columns)."""
    from lib.pointcloud import pcd_from_np

    arr = np.array([[1.0, 2.0, 3.0, 128], [4.0, 5.0, 6.0, 255]], dtype=np.float64)
    pcd = pcd_from_np(arr, columns="XYZI", estimate_normals=False)
    assert len(pcd.points) == 2
    pts = np.asarray(pcd.points)
    assert np.allclose(pts[0], [1.0, 2.0, 3.0])
