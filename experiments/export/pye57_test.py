import open3d as o3d
from lib.pointcloud import save_pointcloud


DemoICPPointClouds = o3d.data.DemoICPPointClouds()
pcd = o3d.io.read_point_cloud(DemoICPPointClouds.paths[0])
save_pointcloud(pcd, "export/cloud_bin_0.e57")
