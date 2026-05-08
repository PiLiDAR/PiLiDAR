import time
import math
import numpy as np
import threading
from mpu6050 import MPU6050


class MPU6050Wrapper:
    def __init__(self, i2c_bus=3, device_address=0x68, freq=50):
        self.i2c_bus = i2c_bus
        self.device_address = device_address
        self.freq_divider = int(200 / freq)  # 0x04

        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.freq_divider)
        self.mpu.reset()   # reset device to clear any previous state before DMP init
        time.sleep(0.1)
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        
        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_buffer = [0]*64

        self.quat = None  # current Quaternion object
        
        self.running = True
        self.thread = threading.Thread(target=self._read_mpu6050)
        self.thread.daemon = True
        self.thread.start()
    
    def _read_mpu6050(self):
        while self.running:
            try:
                if self.mpu.isreadyFIFO(self.packet_size):
                    self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
                    quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)

                    # Check if any value is NaN
                    if not any(math.isnan(value) for value in [quat.w, quat.x, quat.y, quat.z]):
                        self.quat = quat
                        time.sleep(0.02)
                    else:
                        time.sleep(0.01)
            except OSError as e:
                time.sleep(0.01)
    
    def get_euler_angles(self):
        return self.mpu.DMP_get_euler_roll_pitch_yaw(self.quat)
    
    def get_quat_values(self):
        return np.array([self.quat.w, self.quat.x, self.quat.y, self.quat.z])
    
    def close(self):
        self.running = False
        self.thread.join(timeout=1)


if __name__ == '__main__':
    from config import Config # , format_value

    config = Config()
    config.init()

    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"), config.get("IMU", "frequency"))
    
    try:
        time.sleep(0.5)  # wait for FIFO buffer to fill
        while True:
            quat_float = imu.get_quat_values() / 16384
            print(f'\r Quaternion (wxyz): { quat_float}', end='')

            # euler = imu.get_euler_angles()
            # print(f'\r Euler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}', end='')
            time.sleep(0.01)

    except KeyboardInterrupt:
        imu.close()
        print("\nStopped.")
