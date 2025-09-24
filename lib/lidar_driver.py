"""LiDAR driver for the LDRobot STL27L (Waveshare).

The original project already translated the binary data stream into numpy
arrays.  This rewritten version keeps that behaviour but now stores *all*
available metadata: motor speed, timestamps, the vertical platform angle and
the raw intensity values.  The extra information is essential for building a
fully traceable data set and allows additional post processing steps later on.

The implementation is intentionally verbose with many comments so beginners can
follow every step of the decoding process.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

import numpy as np
import serial

try:  # pragma: no cover - executed when used inside the project package
    from lib.config import Config
    from lib.platform_utils import init_serial
    from lib.pointcloud import get_scan_dict, save_raw_scan
except Exception:  # pragma: no cover - fallback for interactive testing
    from config import Config  # type: ignore
    from platform_utils import init_serial  # type: ignore
    from pointcloud import get_scan_dict, save_raw_scan  # type: ignore


@dataclass
class PackageData:
    """Container that stores a single LiDAR package."""

    timestamp: float
    speed: float
    angles_rad: np.ndarray
    distances_mm: np.ndarray
    intensities: np.ndarray
    cartesian: np.ndarray
    z_angle: Optional[float]


class Lidar:
    """High level helper around the serial protocol of the STL27L LiDAR."""

    def __init__(
        self,
        config: Config,
        visualization=None,
        serial_connection=None,
        status_callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        self.config = config
        self.visualization = visualization
        self.status_callback = status_callback

        self.verbose = False
        self.sampling_rate = config.get("LIDAR", config.DEVICE, "SAMPLING_RATE")
        self.raw_path = config.raw_path

        self.z_angle: Optional[float] = None  # gets updated externally by A4988 driver

        # constants describing the binary protocol
        self.start_byte = bytes([0x54])
        self.dlength_byte = bytes([0x2C])
        self.dlength = 12  # 12 samples per package
        self.package_len = 47  # start_byte + dlength_byte + 44 byte payload + 1 byte CRC
        self.deg2rad = np.pi / 180
        self.offset = config.get("LIDAR", config.DEVICE, "OFFSET")
        self.crc_table = config.crc_table

        # Serial connection
        self.port = config.PORT
        baudrate = config.get("LIDAR", config.DEVICE, "BAUDRATE")
        self.serial_connection = serial_connection or init_serial(port=self.port, baudrate=baudrate)

        # command bytes for motor control
        self.start_command = getattr(config, "lidar_start_command", "1")
        self.stop_command = getattr(config, "lidar_stop_command", "0")

        # runtime flags
        self.powered = False
        self.motor_running = False
        self.stop_requested = False

        # buffers used while decoding the packages
        self.byte_array = bytearray()
        self.dtype = np.float32
        self.out_len = config.get("LIDAR", config.DEVICE, "OUT_LEN")

        self.timestamp = 0.0
        self.speed = 0.0
        self.angle_package = np.zeros(self.dlength)
        self.distance_package = np.zeros(self.dlength)
        self.luminance_package = np.zeros(self.dlength)

        self.out_i = 0
        self.speeds = np.empty(self.out_len, dtype=self.dtype)
        self.timestamps = np.empty(self.out_len, dtype=self.dtype)
        self.points_2d = np.empty((self.out_len * self.dlength, 3), dtype=self.dtype)
        self.polar_points = np.empty((self.out_len * self.dlength, 3), dtype=self.dtype)

        # collected output across the full scan
        self.z_angles: List[float] = []
        self.cartesian_list: List[np.ndarray] = []
        self.angular_list: List[np.ndarray] = []
        self.package_history: List[PackageData] = []
        self.stepper_log: List[Dict[str, float]] = []

    # ------------------------------------------------------------------
    # helper and housekeeping methods
    # ------------------------------------------------------------------
    def set_status_callback(self, callback: Callable[[str], None]) -> None:
        self.status_callback = callback

    def _notify(self, message: str) -> None:
        if self.status_callback is not None:
            self.status_callback(message)

    def power_on(self) -> None:
        if self.powered:
            return
        self.config.lidar_power_on()
        self.powered = True
        self._notify("LiDAR Strom eingeschaltet.")

    def power_off(self) -> None:
        if not self.powered:
            return
        self.config.lidar_power_off()
        self.powered = False
        self._notify("LiDAR Strom ausgeschaltet.")

    def start_motor(self) -> None:
        if self.motor_running or self.serial_connection is None:
            return
        try:
            self.serial_connection.write(self.start_command.encode())
            self.motor_running = True
            self._notify("LiDAR-Motor gestartet.")
        except (serial.SerialException, AttributeError):
            self._notify("Konnte den LiDAR-Motor nicht starten (Seriell nicht verfügbar).")

    def stop_motor(self) -> None:
        if not self.motor_running or self.serial_connection is None:
            return
        try:
            self.serial_connection.write(self.stop_command.encode())
        except (serial.SerialException, AttributeError):
            pass
        finally:
            self.motor_running = False
            self._notify("LiDAR-Motor gestoppt.")

    def request_stop(self) -> None:
        self.stop_requested = True

    def close(self) -> None:
        """Gracefully shut down serial communication and power."""

        self.stop_motor()
        self.power_off()
        if self.visualization is not None:
            self.visualization.close()
            print("Visualization closed.\n")

        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Serial connection closed.\n")

    # ------------------------------------------------------------------
    # reading the serial stream
    # ------------------------------------------------------------------
    def read_loop(self, callback=None, max_packages=None, digits=4):
        loop_count = 0
        self.stop_requested = False
        if self.visualization is not None:
            # matplotlib close event
            def on_close(event):
                self.serial_connection.close()
                print("Closing...")

            self.visualization.fig.canvas.mpl_connect('close_event', on_close)

        while (
            self.serial_connection.is_open
            and (max_packages is None or loop_count <= max_packages)
            and not self.stop_requested
        ):
            try:
                if self.out_i == self.out_len:
                    if callback is not None:
                        callback()

                    self.z_angles.append(self.z_angle)
                    if self.verbose:
                        print("speed:", round(self.speed, 2))
                        if self.z_angle is not None:
                            print("z_angle:", round(self.z_angle, 2))

                    self.cartesian_list.append(np.copy(self.points_2d))
                    self.angular_list.append(np.copy(self.polar_points))

                    if self.visualization is not None:
                        self.visualization.update(self.points_2d)

                    self.out_i = 0

                self.read()

            except serial.SerialException:
                print("SerialException")
                break

            self.out_i += 1
            loop_count += 1

    def read(self):
        # iterate through serial stream until start package is found
        while self.serial_connection.is_open:
            data_byte = self.serial_connection.read()

            if data_byte == self.start_byte:
                # Check if the next byte is the second byte of the start sequence
                next_byte = self.serial_connection.read()
                if next_byte == self.dlength_byte:
                    # If it is, read the entire package
                    self.byte_array = self.serial_connection.read(self.package_len - 2)
                    self.byte_array = self.start_byte + self.dlength_byte + self.byte_array
                    break
                else:
                    # If it's not, discard the current byte and continue
                    continue

        if len(self.byte_array) != self.package_len:
            if self.verbose:
                print("[WARNING] Incomplete package:", self.byte_array)
            self.byte_array = bytearray()
            return

        if not self.check_CRC8(self.byte_array):
            if self.verbose:
                print("[WARNING] Invalid package:", self.byte_array)
            self.byte_array = bytearray()
            return

        self.decode(self.byte_array)
        x_package, y_package = self.polar2cartesian(self.angle_package, self.distance_package, self.offset)
        points_package = np.column_stack((x_package, y_package, self.luminance_package)).astype(self.dtype)
        polar_package = np.column_stack((self.angle_package, self.distance_package, self.luminance_package)).astype(self.dtype)

        self.speeds[self.out_i] = self.speed
        self.timestamps[self.out_i] = self.timestamp
        start = self.out_i * self.dlength
        end = (self.out_i + 1) * self.dlength
        self.points_2d[start:end] = points_package
        self.polar_points[start:end] = polar_package

        package_entry = PackageData(
            timestamp=self.timestamp,
            speed=self.speed,
            angles_rad=self.angle_package.copy(),
            distances_mm=self.distance_package.copy(),
            intensities=self.luminance_package.copy(),
            cartesian=points_package.copy(),
            z_angle=self.z_angle,
        )
        self.package_history.append(package_entry)

        self.byte_array = bytearray()

    # ------------------------------------------------------------------
    # data conversion helpers
    # ------------------------------------------------------------------
    def decode(self, byte_array):
        self.speed = int.from_bytes(byte_array[2:4][::-1], 'big') / 360
        FSA = float(int.from_bytes(byte_array[4:6][::-1], 'big')) / 100
        LSA = float(int.from_bytes(byte_array[42:44][::-1], 'big')) / 100
        self.timestamp = int.from_bytes(byte_array[44:46][::-1], 'big')

        angleStep = ((LSA - FSA) if LSA - FSA > 0 else (LSA + 360 - FSA)) / (self.dlength - 1)

        for counter, i in enumerate(range(0, 3 * self.dlength, 3)):
            self.angle_package[counter] = ((angleStep * counter + FSA) % 360) * self.deg2rad
            self.distance_package[counter] = int.from_bytes(byte_array[6 + i:8 + i][::-1], 'big')
            self.luminance_package[counter] = byte_array[8 + i]

    @staticmethod
    def polar2cartesian(angles, distances, offset):
        angles = list(np.array(angles) + offset)
        x_list = distances * -np.cos(angles)
        y_list = distances * np.sin(angles)
        return x_list, y_list

    def check_CRC8(self, data, crc=None):
        def split_last_byte(data):
            return data[:-1], data[-1]

        if crc is None:
            data, crc = split_last_byte(data)

        calculated_crc = 0
        for byte in data:
            if not 0 <= byte <= 255:
                raise ValueError(f"Invalid byte value: {byte}")
            calculated_crc = self.crc_table[(calculated_crc ^ byte) & 0xFF]

        return calculated_crc == crc

    # ------------------------------------------------------------------
    # raw data export helpers
    # ------------------------------------------------------------------
    def create_raw_scan(self, metadata=None):
        packages = [
            {
                "timestamp": pkg.timestamp,
                "speed": pkg.speed,
                "angles_rad": pkg.angles_rad,
                "distances_mm": pkg.distances_mm,
                "intensities": pkg.intensities,
                "cartesian": pkg.cartesian,
                "z_angle": pkg.z_angle,
            }
            for pkg in self.package_history
        ]

        raw_scan = get_scan_dict(
            self.z_angles,
            angular_list=self.angular_list,
            cartesian_list=self.cartesian_list,
            packages=packages,
            metadata=metadata,
            stepper_log=list(self.stepper_log) if self.stepper_log else None,
        )
        save_raw_scan(self.raw_path, raw_scan)
        return raw_scan


if __name__ == "__main__":
    def my_callback():
        pass

    config = Config()
    config.init(scan_id="demo")

    lidar = Lidar(config, visualization=None)
    digits = config.get("ANGULAR_DIGITS")

    try:
        if lidar.serial_connection.is_open:
            lidar.power_on()
            lidar.start_motor()
            lidar.read_loop(callback=my_callback, max_packages=config.max_packages, digits=digits)
    finally:
        print("speed:", round(lidar.speed, 2))
        lidar.create_raw_scan()
        lidar.close()
