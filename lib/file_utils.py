import os
import csv
import re
import math
import numpy as np
from glob import glob
import threading


def modify_path(input_path, insert_string, ext=None):
    # Split the path into directory, base filename, and extension
    directory, filename = os.path.split(input_path)
    base, filetype = os.path.splitext(filename)

    if ext is not None:
        filetype = ext

    new_filename = f"{base}{insert_string}{filetype}"
    new_path = os.path.join(directory, new_filename)
    return new_path

def save_data(filepath, np_array):
    ext = os.path.splitext(filepath)[1].lstrip('.')

    if ext == 'csv':
        t = threading.Thread(target=save_csv, args=(filepath, np_array))
    else:  # format == 'npy':
        t = threading.Thread(target=save_npy, args=(filepath, np_array))
    t.start()

def save_npy(filepath, points_2d):
    np.save(filepath, points_2d)

def save_csv(filepath, points_2d, csv_delimiter=','):
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f, delimiter=csv_delimiter)
        writer.writerows(points_2d)

def csv_from_npy_dir(dir):
    npy_files = list_files(dir, ext='npy')
    for npy_file in npy_files:
        data = np.load(npy_file)
        csv_file = os.path.splitext(npy_file)[0] + '.csv'
        save_csv(csv_file, data)

def list_files(dir, ext=None, filter=None, recursive=False):
    # from glob import glob
    # return sorted(glob(os.path.join(dir, '*.'+ext)))

    filepaths = list()
    for root, dirs, files in os.walk(dir, topdown=True):
        files = sorted(files)

        for file in files:
            if filter is None or filter in file:
                if ext is None or os.path.splitext(file)[1][1:] == ext:  # [1:] just removes the dot in ".jpg"
                    filepath = os.path.join(root, file)
                    filepaths.append(filepath)

        if not recursive:
            break
    print(f"found: {len(filepaths)} {ext} files ({len(files)-len(filepaths)} other)")
    return filepaths

def make_dir(dir):
    # if not os.path.exists(dir):
    os.makedirs(dir, exist_ok=True)
    return dir

def angles_from_filenames(data_dir, name="plane", ext="jpg", filter_NaN=True):
    # create a list of files
    files = glob(os.path.join(data_dir, f"*.{ext}"))
    files = sorted(files)

    angles = []
    for file in files:
        fname = os.path.basename(file)
        # use a regular expression to find the angle value
        angle = re.search(rf"{name}_(-?\d+\.\d+)", fname)
        if angle:
            angles.append(float(angle.group(1)))
        else:
            angles.append(math.nan)

    angles = np.array(angles)

    # sort both lists by the angles list
    files, angles = zip(*sorted(zip(files, angles), key=lambda pair: pair[1]))

    if filter_NaN:
        # filter out the lines that contain a NaN value in the angles list
        files, angles = zip(*filter(lambda pair: not math.isnan(pair[1]), zip(files, angles)))

    return files, angles


if __name__ == "__main__":

    DATA_DIR = "scans/240824-1230/lidar"

    # # CONVERT ALL .npy FILES TO .csv
    # csv_from_npy_dir(DATA_DIR)

    # extract angles from filenames
    files, angles = angles_from_filenames(DATA_DIR, name="plane", ext="npy")

    for i, (file, angle) in enumerate(zip(files, angles)):
        print(f"{i:03d}: {file}\t -> {angle:.2f}")
