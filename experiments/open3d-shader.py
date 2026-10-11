'''
https://stackoverflow.com/questions/69478723/transparency-channel-for-point-cloud-for-open3d
'''

import numpy as np
import open3d as o3d

# Create three random point clouds
rng = np.random.default_rng()
pts1 = rng.random((100, 3), dtype=np.float32)
pts2 = rng.random((100, 3), dtype=np.float32)
pts3 = rng.random((100, 3), dtype=np.float32)
t1 = o3d.core.Tensor(pts1)
t2 = o3d.core.Tensor(pts2)
t3 = o3d.core.Tensor(pts3)
pcd1 = o3d.t.geometry.PointCloud(t1)
pcd2 = o3d.t.geometry.PointCloud(t2)
pcd3 = o3d.t.geometry.PointCloud(t3)

mat1 = o3d.visualization.rendering.MaterialRecord()
mat1.shader = 'defaultLitTransparency'
mat1.base_color = [1.0, 0.0, 0.0, 0.5]  # RGBA, adjust A for transparency
mat1.point_size = 20.0
mat2 = o3d.visualization.rendering.MaterialRecord()
mat2.shader = 'defaultLitTransparency'
mat2.base_color = [0.0, 1.0, 0.0, 0.3]  # RGBA, adjust A for transparency
mat2.point_size = 20.0
mat3 = o3d.visualization.rendering.MaterialRecord()
mat3.shader = 'defaultLitTransparency'
mat3.base_color = [0.0, 0.0, 1.0, 0.1]  # RGBA, adjust A for transparency
mat3.point_size = 20.0

clouds = [{'name': 'pcd1', 'geometry': pcd1, 'material': mat1},
          {'name': 'pcd2', 'geometry': pcd2, 'material': mat2},
          {'name': 'pcd3', 'geometry': pcd3, 'material': mat3}]

# o3d.visualization.draw uses open3d.visualization.O3DVisualizer to display the geometries
o3d.visualization.draw(clouds, bg_color=(0.1, 0.1, 0.1, 1.0), show_skybox=False)
