"""Tests for the pre-flight diagnostics module (``lib/diagnostics.py``).

These run on desktop/CI (no Pi): hardware checks must degrade to SKIP rather
than crash, and the pure-logic checks (storage, hugin, result aggregation)
must produce the right statuses.
"""
import os

from lib.diagnostics import FAIL, OK, SKIP, WARN, Result, check_storage, run_doctor


def test_run_doctor_returns_results_for_every_check():
    results = run_doctor(lidar_seconds=0.1)
    names = [r.name for r in results]
    assert names == ["python", "platform", "storage", "hugin", "lidar", "imu", "camera", "gpio"]
    assert all(isinstance(r, Result) for r in results)
    assert all(r.status in (OK, WARN, FAIL, SKIP) for r in results)


def test_run_doctor_hardware_checks_skip_off_pi():
    """On a non-Pi host the four hardware checks must SKIP, never FAIL."""
    results = run_doctor(lidar_seconds=0.1)
    by_name = {r.name: r for r in results}
    for name in ("lidar", "imu", "camera", "gpio"):
        assert by_name[name].status == SKIP, f"{name} should SKIP: {by_name[name].message}"


def test_run_doctor_skip_argument():
    results = run_doctor(lidar_seconds=0.1, skip=["lidar", "imu"])
    by_name = {r.name: r for r in results}
    assert by_name["lidar"].status == SKIP
    assert by_name["imu"].status == SKIP


def test_check_storage_reports_free_space():
    class _Cfg:
        scans_root = os.path.dirname(os.path.abspath(__file__))

    status, message = check_storage(_Cfg())
    assert status in (OK, WARN, FAIL)
    assert "free on" in message
