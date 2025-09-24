import pandas as pd


def read_ascii(filename, **kwargs):
    """Read an ascii file and store elements in pandas DataFrame.

    Parameters
    ----------
    filename: str
        Path to the filename
    kwargs: pandas.read_csv supported kwargs
        Check pandas documentation for all possibilities.
    Returns
    -------
    data: dict
        Elements as pandas DataFrames.
    """

    data = {
        "points": pd.read_csv(filename, **kwargs)
    }

    return data


def write_ascii(filename, points, **kwargs):
    """Write points content to filename.

    Parameters
    ----------
    filename: str
        Path to output filename
    points: pd.DataFrame
        Points data

    kwargs: see pd.DataFrame.to_csv

    Returns
    -------
    bool
    """
    points[["x", "y", "z"]].to_csv(filename, **kwargs)

    return True
