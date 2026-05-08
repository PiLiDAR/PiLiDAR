'''
USAGE:
    python process_3D.py <scan_id> [options]

    Arguments:
        scan_id             Scan ID to process (e.g. 260508-2004)

    Options:
        --pano   true|false     Enable panorama stitching     (default: false)
        --colour true|false     Assign vertex colours         (default: true)
        --filter true|false     Enable outlier point filtering (default: false)

    Example:
        python process_3D.py 260508-2010 --pano true --filter true 
'''

import time
import argparse
import glob
import os
from lib.config import Config
from lib.visualization import visualize
from lib.pointcloud import process_raw
from lib.pano_utils import hugin_stitch


def str_to_bool(v):
    if v.lower() in ('true', '1', 'yes'):
        return True
    elif v.lower() in ('false', '0', 'no'):
        return False
    raise argparse.ArgumentTypeError("Expected true/false")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Process raw LiDAR scan into a 3D point cloud.")
    parser.add_argument("scan_id",   help="Scan ID to process (e.g. 260508-2004)")
    parser.add_argument("--pano",    type=str_to_bool, default=False, metavar="true|false", help="Enable panorama stitching (default: false)")
    parser.add_argument("--colour",  type=str_to_bool, default=True,  metavar="true|false", help="Assign vertex colours (default: true)")
    parser.add_argument("--filter",  type=str_to_bool, default=False, metavar="true|false", help="Enable outlier point filtering (default: false)")
    args = parser.parse_args()

    config = Config()
    config.init(scan_id=args.scan_id)

    config.set(True,          "ENABLE_3D")
    config.set(args.pano,     "ENABLE_PANO")
    config.set(args.colour,   "ENABLE_VERTEXCOLOUR")
    config.set(args.filter,   "ENABLE_FILTERING")

    # PANORAMA STITCHING
    if args.pano:
        config.imglist = sorted(glob.glob(os.path.join(config.img_dir, "*.jpg")))
        if not config.imglist:
            print("No images found for panorama stitching.")
        else:
            print(f"\nStitching panorama from {len(config.imglist)} images...")
            t_pano = time.perf_counter()
            hugin_stitch(config)
            print(f"Panorama stitching completed. ({time.perf_counter()-t_pano:.1f}s)")

    # PROCESS RAW DATA FROM PKL FILE
    t0 = time.perf_counter()
    pcd = process_raw(config, save=True)
    print(f"\nTotal processing time: {time.perf_counter()-t0:.1f}s")

    # VISUALIZATION
    if config.platform != 'RaspberryPi':
        visualize(pcd, view="front", unlit=True, point_size=1, fullscreen=True)
