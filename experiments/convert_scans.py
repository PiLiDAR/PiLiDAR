"""Convert legacy pickled raw scans to the open container (NPZ + JSON).

For every ``scans/<id>/<id>_lidar.pkl`` this writes ``<id>_lidar.npz`` +
``<id>_lidar.json`` (see ``lib.pointcloud.save_raw_scan``). The original ``.pkl``
is left in place; ``load_raw_scan`` prefers the ``.npz`` from then on.

Usage:
    ~/venv/bin/python experiments/convert_scans.py            # all scans
    ~/venv/bin/python experiments/convert_scans.py 261011-2329
    ~/venv/bin/python experiments/convert_scans.py --force    # overwrite existing .npz
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from lib.pointcloud import convert_raw_scan  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def convert(pkl_path, force=False):
    return "ok" if convert_raw_scan(pkl_path, force=force) else "skip"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scan_ids", nargs="*", help="scan ids (default: all)")
    ap.add_argument("--force", action="store_true", help="overwrite existing .npz")
    args = ap.parse_args()

    if args.scan_ids:
        picks = [os.path.join(ROOT, "scans", s, f"{s}_lidar.pkl") for s in args.scan_ids]
    else:
        picks = glob.glob(os.path.join(ROOT, "scans", "*", "*_lidar.pkl"))

    n_ok = n_skip = n_missing = 0
    for pkl in sorted(picks):
        if not os.path.exists(pkl):
            print(f"[missing] {pkl}")
            n_missing += 1
            continue
        try:
            status = convert(pkl, force=args.force)
        except Exception as e:                                  # noqa: BLE001
            print(f"[error]   {os.path.relpath(pkl, ROOT)}: {type(e).__name__}: {e}")
            continue
        print(f"[{status:>4}] {os.path.relpath(pkl, ROOT)}")
        n_ok += status == "ok"
        n_skip += status == "skip"

    print(f"\ndone: {n_ok} converted, {n_skip} already had .npz, {n_missing} missing")


if __name__ == "__main__":
    main()
