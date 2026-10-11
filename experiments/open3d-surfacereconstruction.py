"""
http://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html

normals: http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html
"""

import open3d as o3d

bunny = o3d.data.BunnyMesh()
mesh  = o3d.io.read_triangle_mesh(bunny.path)

pcd = mesh.sample_points_poisson_disk(750)
o3d.visualization.draw_geometries([pcd])

alpha = 0.03
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
