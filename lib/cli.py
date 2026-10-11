"""PiLiDAR command-line interface.

Unified entry point for the previously scattered root scripts. Commands are
mounted as sub-parsers and each delegates to the existing library code:

    pilidar doctor    pre-flight hardware diagnostics            (lib/diagnostics)
    pilidar capture   run a scan                                 (PiLiDAR.py)
    pilidar process   assemble + colour + level a scan's cloud   (lib/pointcloud)
    pilidar export    convert a point cloud (PLY/PCD/...)        (lib/pointcloud)
    pilidar convert   legacy .pkl raw scans -> open NPZ+JSON     (lib/pointcloud)
    pilidar register  multi-scan alignment                       (lib/registration)

Only ``doctor``, ``process`` and ``convert`` are fully wired today; the others
give clear "not yet wired" errors so the command surface is stable while the
implementations land (roadmap Phases 3/5).
"""
from __future__ import annotations

import argparse

from lib.version import __version__ as VERSION


def build_parser():
    parser = argparse.ArgumentParser(prog="pilidar", description="PiLiDAR scanner toolkit")
    parser.add_argument("--version", action="version", version=f"pilidar {VERSION}")
    sub = parser.add_subparsers(dest="command", metavar="<command>")

    p = sub.add_parser("doctor", help="pre-flight hardware diagnostics")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    p.add_argument("--lidar-seconds", type=float, default=3.0, help="seconds to sample the LiDAR UART")
    p.add_argument("--skip", nargs="*", default=[], help="checks to skip: lidar imu camera gpio")

    p = sub.add_parser("capture", help="run a scan (Raspberry Pi only)")
    p.add_argument("--scan-id", default=None, help="scan id (default: current timestamp)")

    p = sub.add_parser("process", help="assemble + colour + level a raw scan into a point cloud")
    p.add_argument("scan_id", help="scan id, e.g. 261011-2329")
    p.add_argument("--no-save", action="store_true", help="do not write the .ply")
    p.add_argument("--no-level", action="store_true", help="skip IMU gravity leveling")
    p.add_argument("--no-colour", action="store_true", help="skip panorama vertex colouring")

    p = sub.add_parser("export", help="convert a point cloud between formats")
    p.add_argument("input", help="input point cloud (PLY/PCD/...)")
    p.add_argument("output", nargs="?", default=None, help="output path (default: input stem + .pcd)")

    p = sub.add_parser("convert", help="convert legacy .pkl raw scans to the open NPZ+JSON container")
    p.add_argument("scan_ids", nargs="*", help="scan ids (default: all scans)")
    p.add_argument("--force", action="store_true", help="overwrite existing .npz")

    p = sub.add_parser("register", help="align multiple point clouds")
    p.add_argument("inputs", nargs="+", help="input point clouds")
    p.add_argument("-o", "--output", default="registered.ply", help="output path")

    return parser


# ---------------------------------------------------------------------------
# command handlers
# ---------------------------------------------------------------------------

def cmd_doctor(args):
    from lib.diagnostics import _print, run_doctor
    results = run_doctor(lidar_seconds=args.lidar_seconds, skip=args.skip)
    _print(results, use_json=args.json)
    return 1 if any(r.status == "FAIL" for r in results) else 0


def cmd_process(args):
    from lib.config import Config
    from lib.pointcloud import process_raw

    config = Config()
    config.init(scan_id=args.scan_id)
    if args.no_level:
        config.set(False, "IMU", "LEVEL")
    if args.no_colour:
        config.set(False, "ENABLE_VERTEXCOLOUR")

    pcd = process_raw(config, save=not args.no_save)
    print(f"processed {args.scan_id}: {len(pcd.points)} points"
          + ("" if args.no_save else f" -> {config.pcd_path}"))
    return 0


def _scans_root():
    """Resolve the scans root (indirection so tests can inject a temp dir)."""
    from lib.config import Config
    return Config().scans_root


def cmd_convert(args):
    import glob
    import os

    from lib.pointcloud import convert_raw_scan

    root = _scans_root()
    if args.scan_ids:
        picks = [os.path.join(root, s, f"{s}_lidar.pkl") for s in args.scan_ids]
    else:
        picks = glob.glob(os.path.join(root, "*", "*_lidar.pkl"))

    n_ok = n_skip = n_missing = 0
    for pkl in sorted(picks):
        if not os.path.exists(pkl):
            print(f"[missing] {os.path.relpath(pkl, root)}")
            n_missing += 1
            continue
        try:
            ok = convert_raw_scan(pkl, force=args.force)
        except Exception as e:                                   # noqa: BLE001
            print(f"[error]   {os.path.relpath(pkl, root)}: {type(e).__name__}: {e}")
            continue
        print(f"[{'ok' if ok else 'skip':>4}] {os.path.relpath(pkl, root)}")
        n_ok += ok
        n_skip += not ok

    print(f"\ndone: {n_ok} converted, {n_skip} already had .npz, {n_missing} missing")
    return 0


def cmd_export(args):
    import os

    import open3d as o3d

    out = args.output or os.path.splitext(args.input)[0] + ".pcd"
    pcd = o3d.io.read_point_cloud(args.input)
    if pcd.is_empty():
        print(f"[error] could not read points from {args.input}")
        return 1
    ok = o3d.io.write_point_cloud(out, pcd)
    print(f"exported {len(pcd.points)} points -> {out}" if ok else f"[error] write failed: {out}")
    return 0 if ok else 1


def cmd_capture(args):
    raise SystemExit(
        "`pilidar capture` is not wired yet (needs the Pi hardware stack). "
        "Run PiLiDAR.py on the Raspberry Pi for now."
    )


def cmd_register(args):
    raise SystemExit(
        "`pilidar register` is not wired yet (roadmap Phase 5). "
        "See experiments/registration/ for the prototype."
    )


_HANDLERS = {
    "doctor": cmd_doctor,
    "capture": cmd_capture,
    "process": cmd_process,
    "export": cmd_export,
    "convert": cmd_convert,
    "register": cmd_register,
}


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.command:
        parser.print_help()
        return 1
    return _HANDLERS[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
