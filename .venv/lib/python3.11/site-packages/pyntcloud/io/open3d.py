import numpy as np
import pandas as pd


def from_open3d(o3d_data, **kwargs):
    """Create a PyntCloud instance from Open3D's PointCloud/TriangleMesh instance"""
    try:
        import open3d as o3d
    except ImportError:
        raise ImportError("Open3D must be installed. Try `pip install open3d`")

    if not isinstance(o3d_data, (o3d.geometry.PointCloud, o3d.geometry.TriangleMesh)):
        raise TypeError(f"Type {type(o3d_data)} not supported for conversion."
                        f"Expected {o3d.geometry.PointCloud} or {o3d.geometry.TriangleMesh}")

    mesh = None
    if isinstance(o3d_data, o3d.geometry.TriangleMesh):
        mesh = pd.DataFrame(data=np.asarray(o3d_data.triangles),
                            columns=['v1', 'v2', 'v3'])

        points = pd.DataFrame(data=np.asarray(o3d_data.vertices),
                              columns=["x", "y", "z"])

        if o3d_data.vertex_colors:
            colors = (np.asarray(o3d_data.vertex_colors) * 255).astype(np.uint8)
            points["red"] = colors[:, 0]
            points["green"] = colors[:, 1]
            points["blue"] = colors[:, 2]

        if o3d_data.vertex_normals:
            normals = np.asarray(o3d_data.vertex_normals)
            points["nx"] = normals[:, 0]
            points["ny"] = normals[:, 1]
            points["nz"] = normals[:, 2]

    elif isinstance(o3d_data, o3d.geometry.PointCloud):
        points = pd.DataFrame(data=np.asarray(o3d_data.points),
                              columns=["x", "y", "z"])

        if o3d_data.colors:
            colors = (np.asarray(o3d_data.colors) * 255).astype(np.uint8)
            points["red"] = colors[:, 0]
            points["green"] = colors[:, 1]
            points["blue"] = colors[:, 2]

        if o3d_data.normals:
            normals = np.asarray(o3d_data.normals)
            points["nx"] = normals[:, 0]
            points["ny"] = normals[:, 1]
            points["nz"] = normals[:, 2]

    return {
        "points": points,
        "mesh": mesh
    }


def to_open3d(cloud,
              mesh=True,
              colors=True,
              normals=True,
              **kwargs):
    """Convert PyntCloud's instance `cloud` to Open3D's PointCloud/TriangleMesh instance"""
    try:
        import open3d as o3d
    except ImportError:
        raise ImportError("Open3D must be installed. Try `pip install open3d`")

    if mesh and cloud.mesh is not None:
        triangle_mesh = o3d.geometry.TriangleMesh()
        triangle_mesh.triangles = o3d.utility.Vector3iVector(cloud.mesh[["v1", "v2", "v3"]].values)
        triangle_mesh.vertices = o3d.utility.Vector3dVector(cloud.xyz)
        if colors and {'red', 'green', 'blue'}.issubset(cloud.points.columns):
            triangle_mesh.vertex_colors = o3d.utility.Vector3dVector(cloud.points[['red', 'green', 'blue']].values)
        if normals and {'nx', 'ny', 'nz'}.issubset(cloud.points.columns):
            triangle_mesh.vertex_normals = o3d.utility.Vector3dVector(cloud.points[['nx', 'ny', 'nz']].values)
        return triangle_mesh
    else:
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(cloud.xyz)
        if colors and {'red', 'green', 'blue'}.issubset(cloud.points.columns):
            point_cloud.colors = o3d.utility.Vector3dVector(cloud.points[['red', 'green', 'blue']].values)
        if normals and {'nx', 'ny', 'nz'}.issubset(cloud.points.columns):
            point_cloud.normals = o3d.utility.Vector3dVector(cloud.points[['nx', 'ny', 'nz']].values)
        return point_cloud
