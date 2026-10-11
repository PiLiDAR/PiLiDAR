from lib.file_utils import list_files
from lib.pano_utils import hugin_stitch
from lib.config import Config

if __name__ == "__main__":

    config = Config()
    config.init(scan_id="_")

    # Get all jpg files in the directory
    files = list_files(config.scan_dir, ext="jpg")
    print(len(files), "images found.")

    config.imglist = [files[0], files[2], files[4], files[6]]

    hugin_stitch(config)
