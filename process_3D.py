from lib.config import Config
from lib.visualization import visualize
from lib.pointcloud import process_raw


if __name__ == "__main__":

    scan_id = "261015-1839"

    config = Config()
    config.init(scan_id=scan_id)

    # DISABLE TEXTURING AND FILTERING
    config.set(True, "ENABLE_3D")
    config.set(False, "ENABLE_PANO")
    config.set(True, "ENABLE_VERTEXCOLOUR")
    config.set(False, "ENABLE_FILTERING")

    # PROCESS RAW DATA FROM PKL FILE
    pcd = process_raw(config, save=True)
    
    # VISUALIZATION
    if config.platform == 'Windows':
        visualize(pcd, view="front", unlit=True, point_size=1, fullscreen=True)
