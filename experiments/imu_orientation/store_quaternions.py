import numpy as np
from mpu6050 import MPU6050

i2c_bus = 3
device_address = 0x68
freq_divider = 0x04

# Make an MPU6050
mpu = MPU6050(i2c_bus, device_address, freq_divider)

# Initiate your DMP
mpu.dmp_initialize()
mpu.set_DMP_enabled(True)

packet_size = mpu.DMP_get_FIFO_packet_size()
FIFO_buffer = [0]*64

sample_num = 100

# Create numpy array to store quaternions
quat_array = np.zeros((sample_num, 4), dtype=np.int16)

# Sample readings
sample_count = 0
while sample_count < sample_num:
    # Check if FIFO data are ready to use...
    if mpu.isreadyFIFO(packet_size):
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        q_int16 = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        quat_array[sample_count] = [q_int16.w, q_int16.x, q_int16.y, q_int16.z]        
        sample_count += 1

np.savez_compressed('quaternions.npz', quat=quat_array)
#np.save('quaternions.npy', quat_array)
#np.savetxt('quaternions.txt', quat_array, fmt='%d')
