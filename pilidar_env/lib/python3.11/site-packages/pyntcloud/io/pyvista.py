import warnings

import numpy as np
import pandas as pd


def from_pyvista(poly_data, **kwargs):
    """Load a PyntCloud mesh from PyVista's PolyData instance"""
    try:
        import pyvista as pv
    except ImportError:
        raise ImportError("PyVista must be installed. Try `pip install pyvista`")

    if not isinstance(poly_data, pv.PolyData):
        raise TypeError("Type {} not yet supported for conversion.".format(type(poly_data)))

    mesh = None
    if poly_data.faces is not None:
        mesh = poly_data.faces.reshape(-1, 4)
        if not np.all(3 == mesh[:, 0]):
            raise ValueError(
                "This mesh is not triangulated. Try triangulating the mesh before passing to PyntCloud.")
        mesh = pd.DataFrame(data=mesh[:, 1:], columns=['v1', 'v2', 'v3'])

    points = pd.DataFrame(data=poly_data.points, columns=["x", "y", "z"])

    scalars = poly_data.point_data
    for name, array in scalars.items():
        if array.ndim == 1:
            points[name] = array
        elif array.ndim == 2:
            if name == "RGB":
                points["red"] = array[:, 0]
                points["green"] = array[:, 1]
                points["blue"] = array[:, 2]
            elif name == "Normals":
                points["nx"] = array[:, 0]
                points["ny"] = array[:, 1]
                points["nz"] = array[:, 2]
            else:
                for n in range(array.shape[1]):
                    points["{}_{}".format(name, n)] = array[:, n]
        else:
            warnings.warn("Ignoring scalar field {} with ndim > 2 ({})".format(name, array.ndim))

    return {
        "points": points,
        "mesh": mesh
    }


def to_pyvista(cloud, mesh=False, use_as_color=("red", "green", "blue"), **kwargs):
    """Convert PyntCloud's instance `cloud` to PyVista's PolyData instance"""
    try:
        import pyvista as pv
    except ImportError:
        raise ImportError('PyVista must be installed. Try `pip install pyvista`')
    if mesh and cloud.mesh is not None:
        mesh = cloud.mesh[["v1", "v2", "v3"]].values
    else:
        mesh = None
    # Either make point cloud or triangulated mesh
    if mesh is not None:
        # Update cells of PolyData
        types = np.full(len(mesh), 3, dtype=int)
        faces = np.insert(mesh, 0, types, axis=1)
        poly = pv.PolyData(cloud.xyz, faces)
    else:
        poly = pv.PolyData(cloud.xyz)

    avoid = ["x", "y", "z"]
    # add scalar arrays
    if all(c in cloud.points.columns for c in use_as_color):
        colors = cloud.points[list(use_as_color)].values
        poly.point_data["RGB"] = colors
        avoid += list(use_as_color)
    # Add other arrays
    for name in cloud.points.columns:
        if name not in avoid:
            poly.point_data[name] = cloud.points[name]

    return poly
