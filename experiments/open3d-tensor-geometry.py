'''
https://github.com/isl-org/Open3D/discussions/5037
https://github.com/isl-org/Open3D/issues/3305
https://github.com/isl-org/Open3D/issues/3341
'''

import numpy as np

from lib.pointcloud import create_pcd_tensor, load_pointcloud, save_pointcloud
from lib.visualization import visualize

num_points = 10000

pcdt = create_pcd_tensor(np.random.rand(num_points, 3) * 200 - 100,  # random points in [-100, 100]
                         colors=np.random.randint(0, 256, size=(num_points, 3), dtype=np.uint8), 
                         normals=np.random.rand(num_points, 3), 
                         intensities=np.random.rand(num_points, 1))

save_pointcloud(pcdt, "scans/RGBI.ply", ply_ascii=True)

# -----------------------------------------

# Read .ply file
pcdt = load_pointcloud("scans/RGBI.ply", as_tensor=True)
#print(pcdt.point['intensities'])

view = {"zoom": 1., "front": (0, 0, 1), "lookat": None, "up": (0, 1, 0)}
visualize(pcdt, view=view, point_size=5)
