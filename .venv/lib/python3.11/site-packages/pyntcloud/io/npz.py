#       HAKUNA MATATA

import numpy as np
import pandas as pd


def read_npz(filename, points_name="points", mesh_name="mesh"):
    """ Read a .npz file and store all possible elements in pandas DataFrame
    Parameters
    ----------
    filename: str
        Path to the filename
    Returns
    -------
    data: dict
        If possible, elements as pandas DataFrames else input format
    """

    data = {}
    with np.load(filename) as npz:
        data["points"] = pd.DataFrame(npz[points_name])
        if mesh_name in npz:
            data["mesh"] = pd.DataFrame(npz[mesh_name])
    return data


def write_npz(filename, **kwargs):
    """
    Parameters
    ----------
    filename: str
        The created file will be named with this

    kwargs: Elements of the pyntcloud to be saved

    Returns
    -------
    boolean
        True if no problems
    """

    for k in kwargs:
        if isinstance(kwargs[k], pd.DataFrame):
            kwargs[k] = kwargs[k].to_records(index=False)
    np.savez_compressed(filename, **kwargs)
    return True
