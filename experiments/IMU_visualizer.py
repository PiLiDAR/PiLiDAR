# TODO: currently unused

import open3d as o3d
import numpy as np


class IMU_Visualizer:
    def __init__(self):
        # Initialize Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        # Create a 3D box to represent the sensor
        self.box = o3d.geometry.TriangleMesh.create_box(width=0.1, height=0.1, depth=0.1)
        self.box.compute_vertex_normals()
        self.box.paint_uniform_color([0.1, 0.1, 0.7])  # Blue color

        # Add the box to the visualizer
        self.vis.add_geometry(self.box)

    def update_visualization(self, pos, quat):
        # Convert quaternion to rotation matrix
        R = o3d.geometry.get_rotation_matrix_from_quaternion(quat)

        # Apply the translation and rotation to the box
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = pos
        self.box.transform(transformation)

        # Update the visualizer
        self.vis.update_geometry(self.box)
        self.vis.poll_events()
        self.vis.update_renderer()
