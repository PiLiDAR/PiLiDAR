import open3d as o3d
import numpy as np
from quat import XYZVector as V
from quat import Quaternion as Q
from scipy.spatial.transform import Rotation as R


def DMP_get_roll_pitch_yaw(a_quat):
    quat = R.from_quat((a_quat.x,a_quat.y,a_quat.z,a_quat.w))
    rpy = quat.as_euler('xyz', degrees=0)
    return V(rpy[0],rpy[1],rpy[2])

def rad2deg(xyz):
    r2d_factor = 180.0 / np.pi
    x_deg = xyz.x * r2d_factor
    y_deg = xyz.y * r2d_factor
    z_deg = xyz.z * r2d_factor
    return V(x_deg, y_deg, z_deg)

def DMP_get_euler(a_quat):
    psi = np.arctan2(2 * a_quat.x * a_quat.y - 2 * a_quat.w * a_quat.z, 2 * a_quat.w * a_quat.w + 2 * a_quat.x * a_quat.x - 1)
    theta = -np.arcsin(np.clip(2 * a_quat.x * a_quat.z + 2 * a_quat.w * a_quat.y, -1.0, 1.0))
    phi = np.arctan2(2 * a_quat.y * a_quat.z - 2 * a_quat.w * a_quat.x, 2 * a_quat.w * a_quat.w + 2 * a_quat.z * a_quat.z - 1)
    return V(psi, theta, phi)

class Orientation:
    def __init__(self, q_list_int16, degrees=False):
        self.quat_list  = []
        self.euler_list = []
        self.rpy_list   = []

        for q_values in q_list_int16:
            quat = self.create_quaternion(q_values / 16384.0)
            self.quat_list.append(quat)

        for quat in self.quat_list:
            self.euler_list.append(self.get_euler(quat, degrees=degrees))
            self.rpy_list.append(self.get_rpy(quat, degrees=degrees))

    def create_quaternion(self, q_values):
        # Convert to quaternion object
        return Q(q_values[0], q_values[1], q_values[2], q_values[3])

    @staticmethod
    def get_euler(quat, degrees=False):
        # Convert to Euler angles (psi, theta, phi)
        euler = DMP_get_euler(quat)
        if degrees:
            euler = rad2deg(euler)
        return euler
    
    @staticmethod
    def get_rpy(quat, degrees=False):
        # Convert to RPY angles
        rpy = DMP_get_roll_pitch_yaw(quat)
        if degrees:
            rpy = rad2deg(rpy)
        return rpy


if __name__ == '__main__':

    visualize = True

    if visualize:
        arrows = []

        def euler_to_rotation_matrix(euler):
            """Convert Euler angles to a rotation matrix."""
            return R.from_euler('xyz', [euler.x, euler.y, euler.z]).as_matrix()

        def create_arrow(length=1.0, radius=0.02, resolution=4, color=[1, 0, 0]):
            """Create an arrow mesh."""
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=radius,
                cone_radius=radius * 1.5,
                cylinder_height=length,
                cone_height=length * 0.2,
                resolution=resolution,
                cylinder_split=4,
                cone_split=1
            )
            arrow.paint_uniform_color(color)
            return arrow


    q_list_int16 = np.load('scans/quaternion_test.npz')['quat']  # TODO: load from raw_data["quaternion"] instead
    orientation = Orientation(q_list_int16, degrees=True)
    

    for i, euler_angles in enumerate(orientation.euler_list):
        print(f"Euler {i}: {euler_angles.x:.3} {euler_angles.y:.3} {euler_angles.z:.3}")

        if visualize:
            rotation_matrix = euler_to_rotation_matrix(euler_angles)
            arrow = create_arrow()
            arrow.rotate(rotation_matrix, center=(0, 0, 0))
            arrows.append(arrow)

    if visualize:
        # Visualize the arrows using Open3D
        o3d.visualization.draw_geometries(arrows)
