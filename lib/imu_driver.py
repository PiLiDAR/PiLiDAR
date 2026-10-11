import time
import math
import numpy as np
import threading
import smbus
from mpu6050 import MPU6050

# MPU6050 registers for a fast, block-based FIFO read
_FIFO_COUNTH = 0x72
_FIFO_R_W = 0x74
_SMBUS_BLOCK_MAX = 32          # I2C_SMBUS_BLOCK_MAX
_QUAT_NORM = 16384.0           # unit quaternion in DMP int16 units
_NORM_TOL = 0.10               # accept +-10% of the unit norm


class MPU6050Wrapper:
    def __init__(self, i2c_bus=3, device_address=0x68, freq=50, retries=10, retry_delay=0.25):
        self.i2c_bus = i2c_bus
        self.device_address = device_address
        self.freq_divider = int(200 / freq)  # 0x04

        # The DMP firmware upload intermittently fails with an I2C EIO/EREMOTEIO
        # even though the chip answers at its address (observed on the Pi). A
        # single glitch would otherwise silently disable the IMU + leveling for
        # the whole scan, so retry a few times before giving up.
        last_error = None
        for _ in range(max(1, retries)):
            try:
                self.mpu = MPU6050(self.i2c_bus, self.device_address, self.freq_divider)
                self.mpu.dmp_initialize()
                self.mpu.set_DMP_enabled(True)
                break
            except OSError as e:
                last_error = e
                time.sleep(retry_delay)
        else:
            raise last_error

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_buffer = [0]*64

        # our own bus handle for bulk FIFO reads (the library reads one byte at
        # a time, which is far too slow over the bit-banged i2c-gpio bus: the
        # 1024-byte FIFO overflows mid-packet and the stream loses alignment)
        self.bus = smbus.SMBus(i2c_bus)

        self.quat = None  # current Quaternion object
        self._lock = threading.Lock()

        self.running = True
        self.thread = threading.Thread(target=self._read_mpu6050)
        self.thread.daemon = True
        self.thread.start()

    def _fifo_count(self):
        hi, lo = self.bus.read_i2c_block_data(self.device_address, _FIFO_COUNTH, 2)
        return (hi << 8) | lo

    def _read_fifo(self, n):
        """Read ``n`` bytes from the FIFO in <=32-byte block transactions."""
        out = []
        while n > 0:
            chunk = min(n, _SMBUS_BLOCK_MAX)
            out.extend(self.bus.read_i2c_block_data(self.device_address, _FIFO_R_W, chunk))
            n -= chunk
        return out

    @staticmethod
    def _valid(quat):
        """A real DMP quaternion has a norm near 16384 (int16 units)."""
        norm = math.sqrt(quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2)
        return abs(norm - _QUAT_NORM) <= _QUAT_NORM * _NORM_TOL

    def _read_mpu6050(self):
        while self.running:
            try:
                # drain the FIFO with bulk reads; keep the freshest valid quaternion
                fifo_count = self._fifo_count()
                if fifo_count >= self.packet_size and fifo_count < 1024:
                    got_packet = False
                    while fifo_count >= self.packet_size:
                        self.FIFO_buffer = self._read_fifo(self.packet_size)
                        quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
                        fifo_count -= self.packet_size

                        if self._valid(quat):
                            with self._lock:
                                self.quat = quat
                            got_packet = True
                        else:
                            # byte misalignment (FIFO overran mid-read): reset to
                            # re-sync rather than feeding garbage downstream
                            self.mpu.reset_FIFO()
                            break

                    time.sleep(0.02 if got_packet else 0.005)
                elif fifo_count >= 1024:
                    # overflowed (1024 bytes is the chip's FIFO size) -> reset
                    self.mpu.reset_FIFO()
                    time.sleep(0.01)
                else:
                    time.sleep(0.005)
            except OSError:
                time.sleep(0.01)

    def get_euler_angles(self):
        with self._lock:
            if self.quat is None:
                return None
            return self.mpu.DMP_get_euler_roll_pitch_yaw(self.quat)

    def get_quat_values(self):
        with self._lock:
            if self.quat is None:
                return None
            return np.array([self.quat.w, self.quat.x, self.quat.y, self.quat.z])

    def close(self):
        self.running = False
        if self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join(timeout=0.5)


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
