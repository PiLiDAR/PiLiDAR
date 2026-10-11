import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# Function to transform IMU data
def transform_imu_data(euler_angles, offset):
    rotation = R.from_euler('xyz', euler_angles, degrees=True)
    transformed_rotation = rotation * offset
    return transformed_rotation.as_euler('xyz', degrees=True)

# Load the JSON file
with open('imu_orientation/euler.json', 'r') as file:
    data = json.load(file)

# Extract the angular values
angular_values = []
for entry in data:
    local_z, euler_x, euler_y, euler_z = entry
    angular_values.append({
        'local_z': local_z,
        'euler_x': euler_x,
        'euler_y': euler_y,
        'euler_z': euler_z
    })

# Calibrate the offset
offsets = []
for value in angular_values:
    local_z = value['local_z']
    euler_angles = [value['euler_x'], value['euler_y'], value['euler_z']]
    device_rotation = R.from_euler('z', -local_z, degrees=True)
    imu_rotation = R.from_euler('xyz', euler_angles, degrees=True)
    offset = device_rotation * imu_rotation.inv()
    offsets.append(offset)

# Compute the average offset using quaternion averaging
quaternions = [offset.as_quat() for offset in offsets]
avg_quaternion = np.mean(quaternions, axis=0)
average_offset = R.from_quat(avg_quaternion)

# Transform the IMU data with the calibrated offset
transformed_values = []
for value in angular_values:
    euler_angles = [value['euler_x'], value['euler_y'], value['euler_z']]
    transformed_euler = transform_imu_data(euler_angles, average_offset)
    transformed_values.append(transformed_euler)

# Average the plane of rotation
average_plane = np.mean(transformed_values, axis=0)
print("Average Plane of Rotation:", average_plane)

# Estimate the orientation at local_z = 0
orientation_at_zero = transform_imu_data([0, 0, 0], average_offset)
print("Orientation at local_z = 0:", orientation_at_zero)