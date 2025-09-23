"""Configuration helper for the PiLiDAR project.

The :class:`Config` class reads :mod:`config.json`, keeps frequently used
values accessible and prepares the directory structure for every scan.  Plenty
of inline comments explain each step so beginners understand what happens when
the code is executed.
"""

from __future__ import annotations

import datetime
import json
import os
from typing import Optional

try:
    from lib.file_utils import make_dir
    from lib.platform_utils import get_platform, allow_serial
except Exception:  # pragma: no cover - fallback for direct module execution
    from file_utils import make_dir  # type: ignore
    from platform_utils import get_platform, allow_serial  # type: ignore


GPIO = None
platform = get_platform()
if platform == 'RaspberryPi':  # pragma: no cover - executed on the real device
    os.environ.setdefault("RPI_LGPIO_REVISION", "0xa020d3")
    import RPi.GPIO as GPIO  # type: ignore


class Config:
    def __init__(self, file_path: str = "config.json", scans_root: Optional[str] = None):
        # ``base_dir`` is the project root directory.  All relative paths inside
        # ``config.json`` are resolved against it so that the code works no
        # matter from where it is launched.
        self.base_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))

        # load config.json
        with open(os.path.join(self.base_dir, file_path), 'r', encoding='utf-8') as file:
            self.dict = json.load(file)

        # ``scans_root`` can be overridden for unit tests.  When no override is
        # provided we read the value from the configuration file.
        if scans_root is not None:
            self.set(scans_root, "SCANS_ROOT")
        self.scans_root = os.path.join(self.base_dir, self.get("SCANS_ROOT"))
        make_dir(self.scans_root)

        # protocol settings contain hexadecimal values that are stored as
        # strings in JSON.  We convert them to raw bytes right away.
        protocol = self.dict["LIDAR"]["protocol"]
        start_byte = bytes.fromhex(protocol["start_byte"])
        self.set(start_byte, "LIDAR", "protocol", "start_byte")
        dlength_byte = bytes.fromhex(protocol["dlength_byte"])
        self.set(dlength_byte, "LIDAR", "protocol", "dlength_byte")

        crc_path = os.path.join(self.base_dir, self.get("LIDAR", "protocol", "CRC_PATH"))
        with open(crc_path, 'r', encoding='utf-8') as file:
            crc_table = json.load(file)["CRC_TABLE"]
        self.crc_table = [int(hex_str, 16) for hex_str in crc_table]
        self.set(self.crc_table, "LIDAR", "protocol", "CRC_TABLE")

        self.platform = get_platform()
        self.GPIO = GPIO
        self.set_device(self.get("LIDAR", "DEVICE"))


        # STEPPER
        self.STEPPER_RES = self.get("STEPPER", "STEPPER_RES")
        self.MICROSTEPS = self.get("STEPPER", "MICROSTEPS")
        self.SCAN_ANGLE = self.get("STEPPER", "SCAN_ANGLE")
        self.stepper_enable_pin = self.get("STEPPER", "pins", "ENABLE_PIN", default=None)

        step_angle = 360 / self.get("STEPPER", "STEPPER_RES")
        self.set(step_angle, "STEPPER", "STEP_ANGLE")

        self.gear_ratio = self.evaluate_formula(self.get("STEPPER", "GEAR_RATIO"))
        self.set(self.gear_ratio, "STEPPER", "GEAR_RATIO")
        self.validate_stepper_settings()

        self.target_res = self.evaluate_formula(self.get("LIDAR", "TARGET_RES"))
        self.update_target_res(self.target_res)


        # PANORAMA
        AEB = self.get("CAM", "AEB")
        AEB_STOPS = self.get("CAM", "AEB_STOPS")
        IMGCOUNT = self.get("PANO", "IMGCOUNT")
        TEMPLATE_DIR = os.path.join(self.base_dir, self.get("PANO", "TEMPLATE_DIR"))
        aeb_name = f"_AEB{AEB}-{AEB_STOPS}" if AEB > 0 else ""

        self.template_path = f"{TEMPLATE_DIR}/template_{IMGCOUNT}{aeb_name}.pto"


    def init(self, scan_id=None):
        ## TODO: fix power control issues
        ## RELAY
        # GPIO.output(relay_pin, 1)  # enable Power relay 
        ## USB HUB
        #bsubprocess.run(["sudo", "uhubctl", "-l", "1-1", "-a", "on"])  # enable USB Hub Power
        ## LIDAR
        # self.serial_connection.write(b'1')  # start STL27L motor
        # self.serial_connection.write(b'0')  # stop STL27L motor
        
        
        if scan_id is not None:
            self.scan_id = scan_id
        else:
            self.scan_id = datetime.datetime.now().strftime("%y%m%d-%H%M")

        # Prepare the directory structure for the current scan.  ``make_dir``
        # silently ignores existing folders so re-running the pipeline is safe.
        self.scan_dir = make_dir(os.path.join(self.scans_root, self.scan_id))
        self.img_dir = make_dir(os.path.join(self.scan_dir, "img"))
        self.tmp_dir = make_dir(os.path.join(self.scan_dir, "tmp"))
        self.logs_dir = make_dir(os.path.join(self.scan_dir, "logs"))

        # File destinations of the generated assets.
        self.pto_path = os.path.join(self.scan_dir, f'{self.scan_id}.pto')
        self.pano_path = os.path.join(self.scan_dir, f'{self.scan_id}{self.get("PANO", "OUTPUT_NAME")}')
        self.raw_path = os.path.join(self.scan_dir, f"{self.scan_id}{self.get('LIDAR', 'RAW_NAME')}")

        ext = self.get("3D", "EXT")
        intensity_suffix = self.get("3D", "INTENSITY_SUFFIX", default="_intensity")
        color_suffix = self.get("3D", "COLOR_SUFFIX", default="_vertex")
        self.intensity_pcd_path = os.path.join(self.scan_dir, f'{self.scan_id}{intensity_suffix}.{ext}')
        self.pcd_path = self.intensity_pcd_path  # backwards compatible alias
        self.vertex_pcd_path = os.path.join(self.scan_dir, f'{self.scan_id}{color_suffix}.{ext}')
        self.filtered_pcd_path = os.path.join(self.scan_dir, f'{self.scan_id}_filtered.{ext}')

        # legacy directory kept for backwards compatibility with earlier data
        # dumps.
        self.lidar_dir = make_dir(os.path.join(self.scan_dir, "lidar"))

        self.imglist = []


    def set_device(self, device: str):
        '''set sampling rate, baudrate and port for selected device'''
        self.DEVICE = device
        self.TARGET_SPEED = self.get("LIDAR", "TARGET_SPEED")

        self.SAMPLING_RATE = self.get("LIDAR", self.DEVICE, "SAMPLING_RATE")
        self.BAUDRATE = self.get("LIDAR", self.DEVICE , "BAUDRATE")
        self.PORT = self.get("LIDAR", self.DEVICE , "PORT")

        self.lidar_power_pin = self.get("LIDAR", "POWER_PIN", default=None)
        self.lidar_start_command = self.get("LIDAR", "MOTOR_START_COMMAND", default="1")
        self.lidar_stop_command = self.get("LIDAR", "MOTOR_STOP_COMMAND", default="0")

        if self.platform == 'RaspberryPi':
            print("Platform: Raspberry Pi")

            # BUG legacy: allow access to serial port on Raspberry Pi
            allow_serial()

            self.gpio_setup()  # enable GPIO Ports

            self.PORT = self.get("LIDAR", self.DEVICE , "PORT")

            # disable filtering on Raspberry Pi as it is computationally too expensive
            if not self.get("FILTERING", "FILTER_ON_PI"):
                self.set(False, "ENABLE_FILTERING")
        
        elif self.platform == 'Windows':
            self.PORT = self.get("LIDAR", self.DEVICE , "PORT_WIN")


    def get(self, *args, default=None):
        value = self.dict
        try:
            for key in args:
                value = value[key]
        except KeyError:
            if default is not None:
                return default
            raise
        return value
    
    def set(self, value, *args):
        d = self.dict
        for key in args[:-1]:
            d = d.setdefault(key, {})
        d[args[-1]] = value


    def update_target_res(self, target_res):
        self.target_res = target_res
        self.set(self.target_res, "LIDAR", "TARGET_RES")

        self.microsteps_per_revolution = self.STEPPER_RES * self.MICROSTEPS * self.gear_ratio   # 11886
        self.steps = int(round(self.microsteps_per_revolution * self.target_res / 360))         # 6
        self.h_res = 360 * self.steps / self.microsteps_per_revolution                          # 0.1817°
        self.horizontal_steps = int(self.SCAN_ANGLE / self.h_res)                               # 990
        self.packages_per_revolution = round(self.SAMPLING_RATE / (12 * self.TARGET_SPEED))     # 180
        self.max_packages = self.horizontal_steps * self.packages_per_revolution                # 178200


    def evaluate_formula(self, formula: str):
        return eval(formula)

    def validate_stepper_settings(self):
        microsteps = self.get("STEPPER", "MICROSTEPS")
        if microsteps not in (1, 2, 4, 8, 16):
            raise ValueError(f"Invalid MICROSTEPS setting: {microsteps}")

        delay = self.get("STEPPER", "STEP_DELAY")
        if not (0 < delay <= 0.1):
            raise ValueError("STEP_DELAY must be between 0 and 0.1 seconds")

        gear_ratio = self.gear_ratio
        if gear_ratio <= 0:
            raise ValueError("GEAR_RATIO must be positive")

        stepper_res = self.get("STEPPER", "STEPPER_RES")
        if stepper_res <= 0:
            raise ValueError("STEPPER_RES must be positive")

    def has_gpio(self) -> bool:
        return self.GPIO is not None

    def gpio_setup(self, debug: bool = False):
        if not self.has_gpio():
            return

        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(debug)

        # power relay
        self.relay_pin = self.get("STEPPER", "RELAY_PIN")
        self.GPIO.setup(self.relay_pin, self.GPIO.OUT)
        self.GPIO.output(self.relay_pin, self.GPIO.HIGH)  # enable power

        if self.stepper_enable_pin is not None:
            self.GPIO.setup(self.stepper_enable_pin, self.GPIO.OUT)
            # High = deaktiviert → verhindert unkontrolliertes Anlaufen beim Booten
            self.GPIO.output(self.stepper_enable_pin, self.GPIO.HIGH)

        if self.lidar_power_pin is not None:
            self.GPIO.setup(self.lidar_power_pin, self.GPIO.OUT)
            self.GPIO.output(self.lidar_power_pin, self.GPIO.LOW)

    def relay_on(self):
        if hasattr(self, "relay_pin") and self.has_gpio():
            self.GPIO.output(self.relay_pin, self.GPIO.HIGH)

    def relay_off(self):
        if hasattr(self, "relay_pin") and self.has_gpio():
            self.GPIO.output(self.relay_pin, self.GPIO.LOW)
            self.GPIO.cleanup(self.relay_pin)

    def lidar_power_on(self):
        if self.lidar_power_pin is not None and self.has_gpio():
            self.GPIO.output(self.lidar_power_pin, self.GPIO.HIGH)

    def lidar_power_off(self):
        if self.lidar_power_pin is not None and self.has_gpio():
            self.GPIO.output(self.lidar_power_pin, self.GPIO.LOW)


def format_value(value, digits):
    try:
        formatted_value = f"{round(value, digits):0{4 + digits}.{digits}f}"
        return formatted_value
    except (TypeError, ValueError):
        return None


if __name__ == "__main__":
    config = Config()
    config.init()

    print("init scan:", config.scan_id)
    # print(config.get("LIDAR", "protocol", "CRC_TABLE"))
