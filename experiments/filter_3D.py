import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lib.pointcloud import load_pointcloud, downsample, filter_outliers, filter_by_reference, save_pointcloud_threaded, print_stats
from lib.visualization import visualize
from lib.config import Config


if __name__ == '__main__':

    scan_id = "240827-1322"

    config = Config()
    config.init(scan_id=scan_id)

    # enable visualization
    vis = True if config.platform == 'Windows' else False

    pcd = load_pointcloud(config.pcd_path, as_tensor=False)
    print_stats(pcd, txt="Initial point cloud:")

    view = "front"
    unlit = True
    fullscreen = True

    if vis:
        visualize(pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)


    if config.get("ENABLE_FILTERING"):
        low_pcd = downsample(pcd, voxel_size=config.get("FILTERING", "VOXEL_SIZE"))
        print_stats(low_pcd, txt="After downsampling:")
        if vis:
            visualize(low_pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)

        nb_points = config.get("FILTERING", "NB_POINTS")
        radius = config.get("FILTERING", "RADIUS")
        filtered_low_pcd = filter_outliers(low_pcd, nb_points=nb_points, radius=radius)
        print_stats(filtered_low_pcd, txt="After removing outliers:")
        if vis:
            visualize(filtered_low_pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)

        filtered_pcd = filter_by_reference(pcd, filtered_low_pcd, radius=radius)
        print_stats(filtered_pcd, txt="Filtered original point cloud using reference:")
        if vis:
            visualize(filtered_pcd, unlit=unlit, point_size=1, fullscreen=fullscreen)

        save_pointcloud_threaded(filtered_pcd, config.filtered_pcd_path)
