import open3d as o3d
import numpy as np

# Create a numpy array of points
points = np.array([
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 1, 1]
])

# Create a point cloud object
point_cloud = o3d.geometry.PointCloud()

# Set the points of the point cloud
point_cloud.points = o3d.utility.Vector3dVector(points)

# Optionally, set colors for the points
colors = np.array([
    [1, 0, 0],  # Red
    [0, 1, 0],  # Green
    [0, 0, 1],  # Blue
    [1, 1, 0],  # Yellow
    [1, 0, 1]   # Magenta
])
point_cloud.colors = o3d.utility.Vector3dVector(colors)

# Debug print
print("Attempting to visualize point cloud...")

# Alternative visualization method
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(point_cloud)
vis.run()
vis.destroy_window()

# Debug print
print("Visualization call completed.")