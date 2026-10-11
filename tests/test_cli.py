"""Tests for the unified `pilidar` CLI (lib/cli.py)."""

import numpy as np
import pytest

from lib.cli import main


def test_version_exits_zero(capsys):
    with pytest.raises(SystemExit) as e:
        main(["--version"])
    assert e.value.code == 0
    assert "pilidar" in capsys.readouterr().out


def test_no_command_prints_help(capsys):
    rc = main([])
    out = capsys.readouterr().out
    assert rc == 1
    for cmd in ("doctor", "capture", "process", "export", "convert", "register"):
        assert cmd in out


def test_convert_roundtrip(tmp_path, monkeypatch):
    """`pilidar convert <id>` writes the open container for a legacy pkl."""
    import pickle

    import lib.cli as cli

    scans_root = tmp_path / "scans"
    scan_dir = scans_root / "test-1"
    scan_dir.mkdir(parents=True)
    raw = {"header": {}, "z_angles": [0.0, 90.0], "cartesian": [np.random.rand(3, 3)] * 2}
    pkl = scan_dir / "test-1_lidar.pkl"
    with open(pkl, "wb") as f:
        pickle.dump(raw, f)

    monkeypatch.setattr(cli, "_scans_root", lambda: str(scans_root))
    rc = main(["convert", "test-1"])
    assert rc == 0
    assert (scan_dir / "test-1_lidar.npz").exists()
    assert (scan_dir / "test-1_lidar.json").exists()


def test_process_not_yet_wired_commands_exit_nonzero():
    for argv in (["capture"], ["register", "a.ply", "b.ply"]):
        with pytest.raises(SystemExit) as e:
            main(argv)
        assert e.value.code != 0


def test_build_parser_has_all_commands():
    from lib.cli import build_parser
    parser = build_parser()
    # parse each command to confirm the sub-parser exists
    assert parser.parse_args(["doctor"]).command == "doctor"
    assert parser.parse_args(["process", "s"]).command == "process"
    assert parser.parse_args(["export", "in.ply"]).command == "export"
    assert parser.parse_args(["convert"]).command == "convert"
    assert parser.parse_args(["register", "a.ply"]).command == "register"
    assert parser.parse_args(["capture"]).command == "capture"
