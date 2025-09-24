import numpy as np
import pandas as pd


def quadrilateral_to_triangular(mesh):
    new_mesh = pd.DataFrame()

    quadrilateral_vertex = mesh[["v1", "v2", "v3", "v4"]].values
    triangular_vertex = np.vstack(
        (quadrilateral_vertex[:, [0, 1, 2]],
         quadrilateral_vertex[:, [2, 3, 0]]))

    new_mesh["v1"] = triangular_vertex[:, 0]
    new_mesh["v2"] = triangular_vertex[:, 1]
    new_mesh["v3"] = triangular_vertex[:, 2]

    if "vn1" in mesh.columns:
        quadrilateral_vertex_normals = mesh[["vn1", "vn2", "vn3", "vn4"]].values
        triangular_vertex_normals = np.vstack(
            (quadrilateral_vertex_normals[:, [0, 1, 2]],
             quadrilateral_vertex_normals[:, [2, 3, 0]]))

        new_mesh["vn1"] = triangular_vertex_normals[:, 0]
        new_mesh["vn2"] = triangular_vertex_normals[:, 1]
        new_mesh["vn3"] = triangular_vertex_normals[:, 2]

    if "vt1" in mesh.columns:
        quadrilateral_vertex_texture = mesh[["vt1", "vt2", "vt3", "vt4"]].values

        triangular_vertex_texture = np.vstack(
            (quadrilateral_vertex_texture[:, [0, 1, 2]],
             quadrilateral_vertex_texture[:, [2, 3, 0]]))

        new_mesh["vt1"] = triangular_vertex_texture[:, 0]
        new_mesh["vt2"] = triangular_vertex_texture[:, 1]
        new_mesh["vt3"] = triangular_vertex_texture[:, 2]

    return new_mesh
