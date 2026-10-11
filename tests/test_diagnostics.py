"""Tests for the pre-flight diagnostics module (``lib/diagnostics.py``).

These run on desktop/CI *and* on the Pi (no hardware is touched): the LiDAR,
IMU, camera and GPIO checks are always passed via ``skip`` so the tests never
open a serial port or claim a pin. The off-Pi SKIP behaviour is asserted only
on a non-Pi host.
"""
import os

import pytest

from lib.diagnostics import FAIL, OK, SKIP, WARN, Result, check_storage, run_doctor
from lib.platform_utils import get_platform

HARDWARE = ["lidar", "imu", "camera", "gpio"]

pytestmark = pytest.mark.skipif(
    os.environ.get("PILIDAR_TEST_HARDWARE") == "1",
    reason="diagnostics tests must not touch hardware",
)


def test_run_doctor_returns_results_for_every_check():
    results = run_doctor(lidar_seconds=0.1, skip=HARDWARE)
    names = [r.name for r in results]
    assert names == ["python", "platform", "storage", "hugin", *HARDWARE]
    assert all(isinstance(r, Result) for r in results)
    assert all(r.status in (OK, WARN, FAIL, SKIP) for r in results)


def test_run_doctor_skip_argument():
    results = run_doctor(lidar_seconds=0.1, skip=HARDWARE)
    by_name = {r.name: r for r in results}
    for name in HARDWARE:
        assert by_name[name].status == SKIP
        assert by_name[name].message == "skipped"


@pytest.mark.skipif(get_platform() == "RaspberryPi", reason="hardware checks run on a Pi")
def test_run_doctor_hardware_checks_skip_off_pi():
    """On a non-Pi host the four hardware checks must SKIP, never FAIL."""
    results = run_doctor(lidar_seconds=0.1)
    by_name = {r.name: r for r in results}
    for name in HARDWARE:
        assert by_name[name].status == SKIP, f"{name} should SKIP: {by_name[name].message}"


def test_check_storage_reports_free_space():
    class _Cfg:
        scans_root = os.path.dirname(os.path.abspath(__file__))

    status, message = check_storage(_Cfg())
    assert status in (OK, WARN, FAIL)
    assert "free on" in message
