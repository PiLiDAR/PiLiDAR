'''
USAGE:
    python filter_3D.py <scan_id>
'''

import time
import argparse
from lib.pointcloud import load_pointcloud, downsample, filter_outliers, filter_by_reference, save_pointcloud_threaded, print_stats
from lib.visualization import visualize
from lib.config import Config


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="Filter an existing point cloud.")
    parser.add_argument("scan_id", help="Scan ID to filter (e.g. 260506-1956)")
    args = parser.parse_args()

    config = Config()
    config.init(scan_id=args.scan_id)
    
    # enable visualization
    vis = True if config.platform == 'Windows' else False
    
    pcd = load_pointcloud(config.pcd_path, as_tensor=False)
    print_stats(pcd, txt="Initial point cloud:")

    view = "front"
    unlit = True
    fullscreen = True
    

    if (config.platform == 'RaspberryPi' and not config.get("FILTERING", "FILTER_ON_PI")):
        print("Filtering on Raspberry Pi is disabled.")
        exit(0)

    if vis:
        visualize(pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)


    if config.get("ENABLE_FILTERING"):
        t0 = time.perf_counter()
        low_pcd = downsample(pcd, voxel_size=config.get("FILTERING", "VOXEL_SIZE"))
        print_stats(low_pcd, txt=f"After downsampling ({time.perf_counter()-t0:.1f}s):")
        if vis:
            visualize(low_pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)

        nb_points = config.get("FILTERING", "NB_POINTS")
        radius = config.get("FILTERING", "RADIUS")
        reference_radius = config.get("FILTERING", "REFERENCE_RADIUS")
        t1 = time.perf_counter()
        filtered_low_pcd = filter_outliers(low_pcd, nb_points=nb_points, radius=radius)
        print_stats(filtered_low_pcd, txt=f"After removing outliers ({time.perf_counter()-t1:.1f}s):")
        if vis:
            visualize(filtered_low_pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)

        t2 = time.perf_counter()
        filtered_pcd = filter_by_reference(pcd, filtered_low_pcd, radius=reference_radius)
        print_stats(filtered_pcd, txt=f"Filtered by reference ({time.perf_counter()-t2:.1f}s):")
        if vis:
            visualize(filtered_pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)

        print(f"Total filtering time: {time.perf_counter()-t0:.1f}s")
        save_pointcloud_threaded(filtered_pcd, config.filtered_pcd_path)
