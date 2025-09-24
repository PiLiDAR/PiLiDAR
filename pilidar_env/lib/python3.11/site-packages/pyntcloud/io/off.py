import pandas as pd
import numpy as np


def read_off(filename):

    with open(filename) as off:

        first_line = off.readline()
        if "OFF" not in first_line:
            raise ValueError('The file does not start whith the word OFF')
        color = True if "C" in first_line else False

        count = 1
        for line in off:
            count += 1
            if line.startswith("#"):
                continue
            line = line.strip().split()
            if len(line) > 1:
                n_points = int(line[0])
                n_faces = int(line[1])
                break

        data = {}
        point_names = ["x", "y", "z"]
        if color:
            point_names.extend(["red", "green", "blue"])

        data["points"] = pd.read_csv(filename, sep=" ", header=None, engine="python",
                                     skiprows=count, skipfooter=n_faces,
                                     names=point_names, index_col=False)
        for n in ["x", "y", "z"]:
            data["points"][n] = data["points"][n].astype(np.float32)

        if color:
            for n in ["red", "green", "blue"]:
                data["points"][n] = data["points"][n].astype(np.uint8)

        data["mesh"] = pd.read_csv(filename, sep=" ", header=None, engine="python",
                                   skiprows=(count + n_points), usecols=[1, 2, 3],
                                   names=["v1", "v2", "v3"])
        return data
