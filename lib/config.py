'''
SCAN_ANGLE: # 180° CW or -180° CCW
GEAR_RATIO: 3.7142857 = 1 + 38/14
STEP_ANGLE: 1.8 = 360° / STEPPER_RES
TARGET_RES: 1/6 = 0.1666667°
OFFSET: 90° -> np.pi / 2

Raspberry Pi 5 Compatibility:
Uses gpiozero library which works on both Pi 4 and Pi 5.
gpiozero uses lgpio as backend on Pi 5 for proper GPIO access.
'''

import json
import os
import datetime
from typing import Optional

try:
    from lib.file_utils import make_dir
    from lib.platform_utils import get_platform, allow_serial
except:
    from file_utils import make_dir
    from platform_utils import get_platform, allow_serial


platform = get_platform()
if platform == 'RaspberryPi':
    from gpiozero import OutputDevice  # type: ignore
else:
    OutputDevice = None  # type: ignore


class Config:
    def __init__(self, file_path: str = "config.json", scans_root: Optional[str] = None):
        self.base_dir = os.path.abspath(os.path.dirname(os.path.dirname(__file__))) # ../config.json

        # load config.json
        with open(os.path.join(self.base_dir, file_path), 'r') as file:
            self.dict = json.load(file)

        # set scans_root if provided, else use config.json
        if scans_root is not None:
            self.set(scans_root, "SCANS_ROOT")
        self.scans_root = os.path.join(self.base_dir, self.get("SCANS_ROOT"))
        
        # write hex strings back to dict:
        protocol = self.dict["LIDAR"]["protocol"]
        # start_byte
        start_byte = bytes.fromhex(protocol["start_byte"])
        self.set(start_byte, "LIDAR", "protocol", "start_byte")
        # dlength_byte
        dlength_byte = bytes.fromhex(protocol["dlength_byte"])
        self.set(dlength_byte, "LIDAR", "protocol", "dlength_byte")

        # CRC_TABLE
        crc_path = os.path.join(self.base_dir, self.get("LIDAR", "protocol", "CRC_PATH"))
        with open(crc_path, 'r') as file:
            crc_table = json.load(file)["CRC_TABLE"]
        self.crc_table = [int(hex_str, 16) for hex_str in crc_table]
        self.set(self.crc_table, "LIDAR", "protocol", "CRC_TABLE")
        
        
        self.platform = get_platform()
        self.set_device(self.get("LIDAR", "DEVICE"))
        
        
        # STEPPER
        self.STEPPER_RES = self.get("STEPPER", "STEPPER_RES")
        self.MICROSTEPS = self.get("STEPPER", "MICROSTEPS")
        self.SCAN_ANGLE = self.get("STEPPER", "SCAN_ANGLE")

        step_angle = 360 / self.get("STEPPER", "STEPPER_RES")
        self.set(step_angle, "STEPPER", "STEP_ANGLE")

        self.gear_ratio = self.evaluate_formula(self.get("STEPPER", "GEAR_RATIO"))
        self.set(self.gear_ratio, "STEPPER", "GEAR_RATIO")

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
        """Initialize scan directory and file paths."""
        if scan_id is not None:
            self.scan_id = scan_id
        else:
            self.scan_id = datetime.datetime.now().strftime("%y%m%d-%H%M")
        
        self.scan_dir           = os.path.join(self.scans_root, self.scan_id)
        self.pto_path           = os.path.join(self.scan_dir, f'{self.scan_id}.pto')
        self.pano_path          = os.path.join(self.scan_dir, f'{self.scan_id}{self.get("PANO", "OUTPUT_NAME")}')
        self.raw_path           = os.path.join(self.scan_dir, f"{self.scan_id}{self.get('LIDAR', 'RAW_NAME')}")
        self.pcd_path = os.path.join(self.scan_dir, f'{self.scan_id}.{self.get("3D", "EXT")}')
        self.filtered_pcd_path = os.path.join(self.scan_dir, f'{self.scan_id}_filtered.{self.get("3D", "EXT")}')
        
        self.img_dir = make_dir(os.path.join(self.scan_dir, "img"))
        self.tmp_dir   = make_dir(os.path.join(self.scan_dir, "tmp"))
        
        self.imglist = []


    def set_device(self, device: str):
        '''set sampling rate, baudrate and port for selected device'''
        self.DEVICE = device
        self.TARGET_SPEED = self.get("LIDAR", "TARGET_SPEED")

        self.SAMPLING_RATE = self.get("LIDAR", self.DEVICE, "SAMPLING_RATE")
        self.BAUDRATE = self.get("LIDAR", self.DEVICE , "BAUDRATE")
        self.PORT = self.get("LIDAR", self.DEVICE , "PORT")

        if self.platform == 'RaspberryPi':
            print("Platform: Raspberry Pi")
            
            # Ensure access to GPIO serial on Raspberry Pi only when enabled
            if self.get("LIDAR", "GPIO_SERIAL", "ENABLE"):
                allow_serial()

            self.gpio_setup()  # enable GPIO Ports

            self.PORT = self.get("LIDAR", self.DEVICE , "PORT")
            if not os.path.exists(self.PORT): 
                # if USB controller board is not connected, try serial port
                self.PORT = self.get("LIDAR", "GPIO_SERIAL", "PORT")
            
            # disable filtering on Raspberry Pi as it is computationally too expensive
            if not self.get("FILTERING", "FILTER_ON_PI"):
                self.set(False, "ENABLE_FILTERING")
        
        elif self.platform == 'Windows':
            self.PORT = self.get("LIDAR", self.DEVICE , "PORT_WIN")
            # disable GPIO-serial settings
            self.set(False, "LIDAR", "GPIO_SERIAL", "ENABLE")


    def get(self, *args, default=None):
        """Get nested config value with optional default."""
        value = self.dict
        try:
            for key in args:
                value = value[key]
            return value
        except (KeyError, TypeError):
            return default
    
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
        """Safely evaluate simple mathematical formulas."""
        # Only allow basic arithmetic operations
        allowed_chars = set('0123456789+-*/(). ')
        if not all(c in allowed_chars for c in formula):
            raise ValueError(f"Invalid characters in formula: {formula}")
        try:
            # Use eval with restricted namespace for safety
            return eval(formula, {"__builtins__": {}}, {})
        except Exception as e:
            raise ValueError(f"Failed to evaluate formula '{formula}': {e}")

    def gpio_setup(self, debug=False):
        # Initialize power relay using gpiozero (compatible with Pi 4 and Pi 5)
        relay_pin = self.get("STEPPER", "RELAY_PIN")
        if OutputDevice is not None:
            self.relay_device = OutputDevice(relay_pin)
        else:
            self.relay_device = None

    def close_gpio(self):
        """Close GPIO resources when done"""
        if hasattr(self, 'relay_device') and self.relay_device is not None:
            self.relay_device.close()


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
