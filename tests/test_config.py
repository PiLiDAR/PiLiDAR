import os

from lib.config import Config


def test_config_creates_scan_directories(tmp_path):
    config = Config(scans_root=str(tmp_path), use_hardware=False)
    config.init()

    assert os.path.isdir(config.scan_dir)
    assert os.path.isdir(config.img_dir)
    assert os.path.isdir(config.tmp_dir)
    assert os.path.isdir(config.logs_dir)
    assert config.raw_path.endswith("_lidar.pkl")
