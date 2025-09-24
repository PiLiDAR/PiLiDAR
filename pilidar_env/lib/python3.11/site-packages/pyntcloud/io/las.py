import numpy as np

try:
    import laspy
except ImportError:
    laspy = None
try:
    import pylas
except ImportError:
    pylas = None
import pandas as pd


def convert_location_to_dtype(data, dtype_str):
    data["points"] = data["points"].astype({"x": dtype_str, "y": dtype_str, "z": dtype_str})
    return data


def get_color_dtype(data, column_names):
    has_color = all(column in data["points"] for column in column_names)
    if has_color:
        color_data_types = [data["points"][column_name].dtype for column_name in column_names]
        if len(set(color_data_types)) > 1:
            raise TypeError(f"Data types of color values are inconsistent: got {color_data_types}")
        color_data_type = color_data_types[0]
    else:
        color_data_type = None
    return color_data_type


def convert_color_to_dtype(data, output_dtype):
    assert output_dtype in ["uint8", "uint16"]
    column_names = ["red", "green", "blue"]
    input_dtype = get_color_dtype(data, column_names)
    if input_dtype is not None:
        # Color information in las/laz files is stored as uint8 or uint16
        if input_dtype not in ["uint8", "uint16"]:
            raise ValueError(f"Invalid color dtype. Expected one of ['uint8', 'uint16'], but got {input_dtype}")
        if input_dtype == "uint8" and output_dtype == "uint16":
            data["points"].loc[:, column_names] *= 256
        elif input_dtype == "uint16" and output_dtype == "uint8":
            data["points"].loc[:, column_names] /= 256
        data["points"] = data["points"].astype(
            {"red": output_dtype, "green": output_dtype, "blue": output_dtype})
    return data


def read_las_with_laspy(filename):
    if laspy is None:
        raise ImportError("laspy (>=2.0) is needed for reading .las files.")
    data = {}
    with laspy.open(filename) as las_file:
        las = las_file.read()
        data["points"] = pd.DataFrame(las.points.array)
        data["points"].columns = (name.lower() for name in data["points"].columns)

        data["points"]["x"] = pd.Series(np.array(las.x))
        data["points"]["y"] = pd.Series(np.array(las.y))
        data["points"]["z"] = pd.Series(np.array(las.z))

        data["las_header"] = las.header
    return data


def read_las_with_pylas(filename):
    data = {}
    if pylas is None:
        raise ImportError("pylas is needed for reading .las files.")
    with pylas.open(filename) as las_file:
        las = las_file.read()
        data["points"] = pd.DataFrame(las.points)
        data["points"].columns = (name.lower() for name in data["points"].columns)

        data["points"]["x"] = pd.Series(np.array(las.x))
        data["points"]["y"] = pd.Series(np.array(las.y))
        data["points"]["z"] = pd.Series(np.array(las.z))

        data["las_header"] = las.header
    return data


def read_las(filename, xyz_dtype="float32", rgb_dtype="uint8", backend="laspy"):
    """Read a .las/laz file and store elements in pandas DataFrame.

    Parameters
    ----------
    filename: str
        Path to the filename
    xyz_dtype: str
        Defines the data type of the xyz coordinate
    rgb_dtype: str
        Defines the data type of the color
    Returns
    -------
    data: dict
        Elements as pandas DataFrames.
    """
    if backend == "pylas":
        data = read_las_with_pylas(filename)
    elif backend == "laspy":
        data = read_las_with_laspy(filename)
    else:
        raise ValueError(f"Unsupported backend. Expected one of ['pylas', 'laspy'] but got {backend}")
    data = convert_location_to_dtype(data, xyz_dtype)
    data = convert_color_to_dtype(data, rgb_dtype)
    return data
