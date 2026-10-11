'''
LiDAR driver for LDRobot LD06 and STL27L (Waveshare)

Sample Rates:
LD06:    4500 samples/s  (375 packages/s x 12 samples/package)
STL27L: 21600 samples/s (1800 packages/s x 12 samples/package)

Speed Control on Raspberry Pi
- RPi hardware PWM: https://pypi.org/project/rpi-hardware-pwm
- curve fitting using scipy.optimize.curve_fit
'''

import numpy as np
import serial

# running from project root
try:
    from lib.config import Config
    from lib.pointcloud import save_raw_scan, get_scan_dict
    from lib.platform_utils import init_serial, init_pwm_Pi  # init_serial_MCU, init_pwm_MCU

# testing from this file
except:
    from config import Config
    from pointcloud import save_raw_scan, get_scan_dict
    from platform_utils import init_serial, init_pwm_Pi  # init_serial_MCU, init_pwm_MCU


class Lidar:
    def __init__(self, config, visualization=None):

        self.verbose            = False
        self.sampling_rate      = config.get("LIDAR", config.DEVICE, "SAMPLING_RATE")
        self.raw_path           = config.raw_path

        self.z_angle            = None  # gets updated externally by A4988 driver

        # constants
        self.start_byte         = bytes([0x54])
        self.dlength_byte       = bytes([0x2c])
        self.dlength            = 12  # 12 samples per package
        self.package_len        = 47  # start_byte + dlength_byte + 44 byte payload, 1 byte CRC
        self.deg2rad            = np.pi / 180
        self.offset             = config.get("LIDAR", config.DEVICE, "OFFSET")
        self.crc_table          = config.crc_table

        # SERIAL
        # dmesg | grep "tty"
        self.port               = config.PORT

        # if self.platform in ['Pico', 'Pico W', 'Metro M7']:
        #     self.serial_connection  = init_serial_MCU(pin=self.port, baudrate=baudrate)
        # else:  # self.platform in ['Windows', 'Linux', 'RaspberryPi']:
        self.serial_connection  = init_serial(port=self.port, baudrate=config.get("LIDAR", config.DEVICE, "BAUDRATE"))


        self.byte_array         = bytearray()
        self.dtype              = np.float32

        self.out_len            = config.get("LIDAR", config.DEVICE, "OUT_LEN")

        # preallocate package:
        self.timestamp          = 0
        self.speed              = 0
        self.angle_package        = np.zeros(self.dlength)
        self.distance_package     = np.zeros(self.dlength)
        self.luminance_package    = np.zeros(self.dlength)
        # preallocate intermediate outputs:
        self.out_i              = 0
        self.speeds             = np.empty(self.out_len, dtype=self.dtype)
        self.timestamps         = np.empty(self.out_len, dtype=self.dtype)
        self.points_2d          = np.empty((self.out_len * self.dlength, 3), dtype=self.dtype)  # [[x, y, l],[..


        # self.data_dir           = config.lidar_dir  # TODO remove -> npy files replaced by single pkl file

        # raw output
        self.z_angles           = []
        self.cartesian_list     = []

        # visualization
        self.visualization      = visualization


        if config.get("LIDAR", "GPIO_SERIAL", "ENABLE"):
            pwm_channel         = config.get("LIDAR", "GPIO_SERIAL", "PWM_CHANNEL")
            pwm_freq            = config.get("LIDAR", "GPIO_SERIAL", "PWM_FREQ")
            self.pwm            = init_pwm_Pi(pwm_channel, frequency = pwm_freq)

            # from speed curve fitting: (slope m, y-intercept b)
            self.pwm_coeffs = config.get("LIDAR", config.DEVICE, "PWM_COEFFS")

            self.target_speed = config.get("LIDAR", "TARGET_SPEED")
            self.pwm_dc     = self.update_speed(self.target_speed)

            self.pwm.start(self.pwm_dc * 100)

        # elif self.platform in ['Pico', 'Pico W', 'Metro M7']:
        #     pwm_pin             = "GP2"
        #     self.pwm            = init_pwm_MCU(pwm_pin, frequency=pwm_frequency)
        #     self.pwm.duty_cycle = int(pwm_dc * 65534)
        else:
            self.pwm = None


    def update_pwm_dc(self, pwm_dc, update=True):
        self.pwm_dc = pwm_dc
        self.pwm.change_duty_cycle(self.pwm_dc * 100)

        # update speed if called directly
        if update:
            self.target_speed = self.speed_from_pwm(self.pwm_dc)

        return self.pwm_dc


    def update_speed(self, target_speed):
        self.target_speed = target_speed
        pwm_dc = self.pwm_from_speed(self.target_speed)
        self.update_pwm_dc(pwm_dc, update=False)
        return pwm_dc


    def close(self):
        if self.pwm is not None:
            self.pwm.stop()
            print("PWM stopped.\n")

        if self.visualization is not None:
            self.visualization.close()
            print("Visualization closed.\n")

        self.serial_connection.close()
        print("Serial connection closed.\n")


    def read_loop(self, callback=None, max_packages=None, digits=4):
        loop_count = 0
        if self.visualization is not None:
            # matplotlib close event
            def on_close(event):
                self.serial_connection.close()
                print("Closing...")

            self.visualization.fig.canvas.mpl_connect('close_event', on_close)

        while self.serial_connection.is_open and (max_packages is None or loop_count <= max_packages):
            try:
                if self.out_i == self.out_len:
                    if callback is not None:
                        callback()

                    # save the z_angle to list
                    self.z_angles.append(self.z_angle)

                    if self.verbose:
                        print("speed:", round(self.speed, 2))
                        if self.z_angle is not None:
                            print("z_angle:", round(self.z_angle, 2))

                    # Append 2D plane to cartesian list. copying avoids identical pointers
                    self.cartesian_list.append(np.copy(self.points_2d))

                    # VISUALIZE
                    if self.visualization is not None:
                        self.visualization.update(self.points_2d)

                    self.out_i = 0

                success = self.read()
                if success:
                    self.out_i += 1
                    loop_count += 1

            except serial.SerialException:
                print("SerialException")
                break


    def read(self):
        # iterate through serial stream until start package is found
        while self.serial_connection.is_open:
            data_byte = self.serial_connection.read()
            if not data_byte:
                return False

            while data_byte == self.start_byte:
                next_byte = self.serial_connection.read()
                if not next_byte:
                    return False
                if next_byte == self.dlength_byte:
                    self.byte_array = self.serial_connection.read(self.package_len - 2)
                    self.byte_array = self.start_byte + self.dlength_byte + self.byte_array
                    break
                # if next_byte is itself a start_byte, re-evaluate without discarding
                data_byte = next_byte

            if self.byte_array:
                break

        # Error handling
        if len(self.byte_array) != self.package_len:
            if self.verbose:
                print("[WARNING] Incomplete package:", self.byte_array)
            self.byte_array = bytearray()
            return False

        # Check if the package is valid using check_CRC8
        if not self.check_CRC8(self.byte_array):
            if self.verbose:
                print("[WARNING] Invalid package:", self.byte_array)
            # If the package is not valid, reset byte_array and continue with the next iteration
            self.byte_array = bytearray()
            return False

        # decoding updates speed, timestamp, angle_package, distance_package, luminance_package
        self.decode(self.byte_array)
        # convert polar to cartesian
        x_package, y_package = self.polar2cartesian(self.angle_package, self.distance_package, self.offset)
        points_package = np.column_stack((x_package, y_package, self.luminance_package)).astype(self.dtype)

        # write into preallocated output arrays at current index
        self.speeds[self.out_i] = self.speed
        self.timestamps[self.out_i] = self.timestamp
        self.points_2d[self.out_i*self.dlength:(self.out_i+1)*self.dlength] = points_package

        # reset byte_array
        self.byte_array = bytearray()
        return True


    def decode(self, byte_array):
        # dlength = 12  # byte_array[46] & 0x1F
        self.speed = int.from_bytes(byte_array[2:4][::-1], 'big') / 360         # rotational frequency in rps
        FSA = float(int.from_bytes(byte_array[4:6][::-1], 'big')) / 100         # start angle in degrees
        LSA = float(int.from_bytes(byte_array[42:44][::-1], 'big')) / 100       # end angle in degrees
        self.timestamp = int.from_bytes(byte_array[44:46][::-1], 'big')         # timestamp in milliseconds < 30000
        # CS = int.from_bytes(byte_array[46:47][::-1], 'big')                   # CRC Checksum, checked even before decoding

        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (self.dlength-1)

        # 3 bytes per sample x 12 samples
        for counter, i in enumerate(range(0, 3 * self.dlength, 3)):
            self.angle_package[counter] = ((angleStep * counter + FSA) % 360) * self.deg2rad
            self.distance_package[counter] = int.from_bytes(byte_array[6 + i:8 + i][::-1], 'big')  # mm units
            self.luminance_package[counter] = byte_array[8 + i]


    def speed_from_pwm(self, pwm):
        m, b = self.pwm_coeffs
        return m * pwm + b


    def pwm_from_speed(self, speed):
        m, b = self.pwm_coeffs
        return (speed - b) / m


    @staticmethod
    def polar2cartesian(angles, distances, offset):
        angles = list(np.array(angles) + offset)
        x_list = distances * -np.cos(angles)
        y_list = distances * np.sin(angles)
        return x_list, y_list



    def check_CRC8(self, data, crc=None):
        '''CRC check: length is 1 Byte, obtained from the verification of all the previous data except itself'''

        def split_last_byte(data):
            return data[:-1], data[-1]

        if crc is None:
            data, crc = split_last_byte(data)

        calculated_crc = 0
        for byte in data:
            if not 0 <= byte <= 255:
                raise ValueError(f"Invalid byte value: {byte}")
            calculated_crc = self.crc_table[(calculated_crc ^ byte) & 0xff]

        return calculated_crc == crc


if __name__ == "__main__":

    def my_callback():
        # print("speed:", round(lidar.speed, 2))
        pass

    config = Config()

    # use LD06 or STL27L
    config.set_device("LD06")

    config.init(scan_id="_")
    visualize = True

    if visualize:
        from matplotlib_2D import plot_2D
        visualization = plot_2D(plotrange=4000, s=1)
    else:
        import threading
        visualization = None

    lidar = Lidar(config, visualization=visualization)
    digits = config.get("ANGULAR_DIGITS")

    try:
        if lidar.serial_connection.is_open:
            if visualize:
                lidar.read_loop(callback=my_callback, max_packages=config.max_packages, digits=digits)
            else:
                read_thread = threading.Thread(target=lidar.read_loop,
                                               kwargs={'callback': my_callback,
                                                       'max_packages': config.max_packages,
                                                       'digits': digits})
                read_thread.start()
                read_thread.join()
    finally:
        print("speed:", round(lidar.speed, 2))

        # Save raw_scan to pickle file
        raw_scan = get_scan_dict(lidar.z_angles, cartesian_list=lidar.cartesian_list)
        save_raw_scan(lidar.raw_path, raw_scan)
        print("Raw scan saved.")

        lidar.close()
