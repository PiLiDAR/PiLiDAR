#       HAKUNA MATATA

import re
import pandas as pd


def read_obj(filename):
    """ Reads and obj file and return the elements as pandas Dataframes.

    Parameters
    ----------
    filename: str
        Path to the obj file.

    Returns
    -------
    Each obj element found as pandas Dataframe.

    """
    v = []
    vn = []
    vt = []
    f = []

    with open(filename) as obj:
        for line in obj:
            if line.startswith('v '):
                v.append(line.strip()[1:].split())

            elif line.startswith('vn'):
                vn.append(line.strip()[2:].split())

            elif line.startswith('vt'):
                vt.append(line.strip()[2:].split())

            elif line.startswith('f'):
                f.append(line.strip()[1:].lstrip())

    points = pd.DataFrame(v, dtype='f4', columns=["x", "y", "z", "w"][:len(v[0])])

    if len(vn) > 0:
        points = points.join(pd.DataFrame(vn, dtype='f4', columns=['nx', 'ny', 'nz']))

    if len(vt) > 0:
        points = points.join(pd.DataFrame(vt, dtype='f4', columns=['u', 'v']))

    data = {"points": points}

    if len(f) < 1:
        return data

    mesh_columns = []
    if f[0].count("//") > 0:
        # wikipedia.org/wiki/Wavefront_.obj_file#Vertex_normal_indices_without_texture_coordinate_indices
        for i in range(f[0].count("//")):
            mesh_columns.append("v{}".format(i + 1))
            mesh_columns.append("vn{}".format(i + 1))
    elif f[0].count("/") > 0:
        if len(vn) > 0:
            # wikipedia.org/wiki/Wavefront_.obj_file#Vertex_normal_indices
            for i in range(f[0].count("/") // 2):
                mesh_columns.append("v{}".format(i + 1))
                mesh_columns.append("vt{}".format(i + 1))
                mesh_columns.append("vn{}".format(i + 1))
        else:
            # wikipedia.org/wiki/Wavefront_.obj_file#Vertex_texture_coordinate_indices
            for i in range(f[0].count("/")):
                mesh_columns.append("v{}".format(i + 1))
                mesh_columns.append("vt{}".format(i + 1))
    else:
        # wikipedia.org/wiki/Wavefront_.obj_file#Vertex_indices
        for i in range(sum(c.isdigit() for c in f[0].split(" "))):
            mesh_columns.append("v{}".format(i + 1))

    mesh = pd.DataFrame([re.split(r'\D+', x) for x in f], dtype='i4', columns=mesh_columns).astype('i4')
    mesh -= 1  # index starts with 1 in obj file

    data["mesh"] = mesh

    return data


def write_obj(filename, points=None, mesh=None):
    """
    Parameters
    ----------
    filename:   str
        The created file will be named with this
    points:     pd.DataFrame
    mesh:       pd.DataFrame

    Returns
    -------
    boolean
        True if no problems

    """
    if not filename.endswith('obj'):
        filename += '.obj'

    if points is not None:
        points = points.copy()
        points = points[["x", "y", "z"]]
        points.insert(loc=0, column="obj_v", value="v")
        points.to_csv(
            filename,
            sep=" ",
            index=False,
            header=False,
            mode='a',
            encoding='ascii')

    if mesh is not None:
        mesh = mesh.copy()
        mesh = mesh[["v1", "v2", "v3"]]
        mesh += 1  # index starts with 1 in obj file
        mesh.insert(loc=0, column="obj_f", value="f")
        mesh.to_csv(
            filename,
            sep=" ",
            index=False,
            header=False,
            mode='a',
            encoding='ascii')

    return True
