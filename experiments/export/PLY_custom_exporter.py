import open3d as o3d
import numpy as np


# Example point cloud data
points = np.random.rand(100, 3)
intensities = np.random.rand(100)

# Create a PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(np.column_stack([intensities, intensities, intensities]))


# TODO: add normals

def save_ply_with_alpha(filename, points, colors, alpha):
    with open(filename, 'w') as f:
        # Write PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("property uchar alpha\n")
        f.write("end_header\n")
        
        # Write vertex data
        for point, color, a in zip(points, colors, alpha):
            f.write(f"{point[0]} {point[1]} {point[2]} {int(color[0]*255)} {int(color[1]*255)} {int(color[2]*255)} {int(a*255)}\n")


# Use the points, colors, and intensities as alpha values
save_ply_with_alpha("export/output_with_alpha.ply", points, np.column_stack([intensities, intensities, intensities]), intensities)

