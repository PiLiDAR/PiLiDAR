import json
import os
import shutil

from pathlib import Path

import pandas as pd

try:
    from IPython.display import IFrame
except ImportError:
    IFrame = None

from .common import get_colors
from ..io.ply import write_ply


def plot_with_threejs(cloud, **kwargs):
    if IFrame is None:
        raise ImportError("IPython is needed for plotting with threejs backend.")

    colors = get_colors(cloud, kwargs["use_as_color"], kwargs["cmap"])

    points = pd.DataFrame(cloud.xyz, columns=["x", "y", "z"])

    for n, i in enumerate(["red", "green", "blue"]):
        points[i] = colors[:, n]

    if kwargs["mesh"] and cloud.mesh is not None:
        mesh = cloud.mesh[["v1", "v2", "v3"]]
    else:
        mesh = None

    ptp = cloud.xyz.ptp()

    BASE_PATH = os.path.dirname(os.path.abspath(__file__))
    src = "{}/{}".format(BASE_PATH, "points.html")
    dst = "{}/{}".format(os.getcwd(), "{}.html".format(kwargs["output_name"]))

    camera_position = (cloud.xyz.max(0) + abs(cloud.xyz.max(0))).tolist()
    look_at = cloud.xyz.mean(0).tolist()

    dest_directory = Path(os.getcwd())
    config_file_path = dest_directory / (kwargs["output_name"] + '.config.json')

    polylines = kwargs["polylines"] or {}
    config_obj = {
        "filename": kwargs["output_name"],
        "camera_position": camera_position,
        "look_at": look_at,
        "point_size": kwargs["initial_point_size"] or ptp / 100,
        "point_opacity": 0.9,
        "polylines_points": list(polylines.values()),
        "polylines_colors": list(polylines.keys()),
    }

    with config_file_path.open('w') as config_file:
        json.dump(config_obj, config_file)

    # write new html file replacing placeholder
    with open(src, "r") as inp, open(dst, "w") as out:
        for line in inp:
            if "FILENAME_PLACEHOLDER" in line:
                line = line.replace("FILENAME_PLACEHOLDER",
                                    "'{}'".format(kwargs["output_name"]))
            out.write(line)

    write_ply("{}.ply".format(kwargs["output_name"]), points=points, mesh=mesh, as_text=True)

    try:
        shutil.copytree("{}/assets".format(BASE_PATH),
                        "{}/{}".format(os.getcwd(), "pyntcloud_plot_assets"))
    except FileExistsError:
        pass

    return IFrame("{}.html".format(kwargs["output_name"]), width=kwargs["width"], height=kwargs["height"])
