from time import sleep
import os
import threading
import time

from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
try:
    from lib.imu_driver import MPU6050Wrapper  # type: ignore
except Exception:
    MPU6050Wrapper = None
from lib.config import Config, format_value
from lib.raw_utils import save_raw_scan, get_scan_dict
from lib.pointcloud_numpy import process_raw as process_raw_numpy
from lib.rpicam_utils import take_HDR_photo, estimate_camera_parameters
from lib.pano_utils import hugin_stitch


print(r'''
 ____  _ _     _ ____    _    ____ 
|  _ \(_) |   (_)  _ \  / \  |  _ \ 
| |_) | | |   | | | | |/ _ \ | |_) | 
|  __/| | |___| | |_| / ___ \|  _ < 
|_|   |_|_____|_|____/_/   \_\_| \_\ 
''')


config = Config()
# Mitigate OpenBLAS munmap warnings by limiting threads on Pi
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
config.init()

# Automatic stepper motor power management
# Turns relay on before movement and schedules power-off after inactivity.
AUTO_MOTOR_IDLE_SECONDS = config.get("STEPPER", "IDLE_OFF_DELAY", default=10) or 10
_motor_idle_timer = None
_motor_power_lock = threading.Lock()

def _power_on_stepper():
    if hasattr(config, 'relay_device') and config.relay_device is not None:
        # Only turn on if currently off
        try:
            config.relay_device.on()
        except Exception:
            pass

def _power_off_stepper():
    if hasattr(config, 'relay_device') and config.relay_device is not None:
        try:
            config.relay_device.off()
            print(f"[AutoPower] Stepper relay powered off after {AUTO_MOTOR_IDLE_SECONDS}s idle.")
        except Exception:
            pass

def _schedule_power_off():
    global _motor_idle_timer
    with _motor_power_lock:
        if _motor_idle_timer is not None:
            _motor_idle_timer.cancel()
        _motor_idle_timer = threading.Timer(AUTO_MOTOR_IDLE_SECONDS, _power_off_stepper)
        _motor_idle_timer.daemon = True
        _motor_idle_timer.start()

def _wrap_stepper_methods(stepper_obj):
    """Monkey-patch movement methods to manage relay power automatically."""
    if not stepper_obj:
        return
    original_move_steps = stepper_obj.move_steps
    original_move_to_angle = stepper_obj.move_to_angle

    def managed_move_steps(*args, **kwargs):
        _power_on_stepper()
        result = original_move_steps(*args, **kwargs)
        _schedule_power_off()
        return result

    def managed_move_to_angle(*args, **kwargs):
        _power_on_stepper()
        result = original_move_to_angle(*args, **kwargs)
        _schedule_power_off()
        return result

    stepper_obj.move_steps = managed_move_steps  # type: ignore
    stepper_obj.move_to_angle = managed_move_to_angle  # type: ignore


enable_cam   = config.get("ENABLE_CAM")
enable_lidar = config.get("ENABLE_LIDAR")
enable_IMU   = config.get("ENABLE_IMU")

# Initialize optional variables
imu = None
quat_list = []
lidar = None

# initialize IMU (optional)
if enable_IMU and MPU6050Wrapper is None:
    print("IMU module not available; disabling IMU features.")
    enable_IMU = False
if enable_IMU and MPU6050Wrapper is not None:
    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"), config.get("IMU", "frequency"))


# initialize stepper
stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"), 
                config.get("STEPPER", "pins", "STEP_PIN"), 
                config.get("STEPPER", "pins", "MS_PINS"), 
                delay=config.get("STEPPER", "STEP_DELAY"),
                step_angle=config.get("STEPPER", "STEP_ANGLE"),
                microsteps=config.get("STEPPER", "MICROSTEPS"),
                gear_ratio=config.get("STEPPER", "GEAR_RATIO"))

# Enable automatic power management for stepper movements
_wrap_stepper_methods(stepper)
# Ensure relay starts off (will turn on at first movement)
try:
    _power_off_stepper()
except Exception:
    pass


# initialize lidar
if enable_lidar:
    lidar = Lidar(config, visualization=None)
    
    # callback function for lidar.read_loop()
    def move_steps_callback() -> None:
        stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
        if lidar is not None:
            lidar.z_angle = stepper.get_current_angle()
        if enable_IMU and imu is not None:
            quat_values = imu.get_quat_values()
            quat_list.append(quat_values)
else:
    def move_steps_callback() -> None:
        pass
            
    if not enable_cam:
        # wait for lidar to lock rotational speed
        sleep(2)

# MAIN
try:
    # 360° SHOOTING PHOTOS
    if enable_cam:
        print("Calibrating Camera...")
        current_exposure_time, current_gain, current_awbgains = estimate_camera_parameters(config)

        IMGCOUNT = config.get("PANO", "IMGCOUNT")
        for i in range(IMGCOUNT):
            current_angle = stepper.get_current_angle()
            formatted_angle = format_value(current_angle, config.get("ANGULAR_DIGITS"))

            print(f"\nTaking photo {i+1}/{IMGCOUNT} | Angle: {formatted_angle}")            
            imgpath = os.path.join(config.img_dir, f"image_{formatted_angle}.jpg")
            
            if enable_IMU and imu is not None:
                euler = imu.get_euler_angles()
                print(f'\tEuler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}')

            # take HDR photo
            imgpaths = take_HDR_photo(AEB           = config.get("CAM", "AEB"), 
                                      AEB_stops     = config.get("CAM", "AEB_STOPS"),
                                      path          = imgpath, 
                                      exposure_time = current_exposure_time, 
                                      gain          = current_gain, 
                                      awbgains      = current_awbgains, 
                                      denoise       = config.get("CAM", "denoise"),
                                      sharpness     = config.get("CAM", "sharpness"),
                                      saturation    = config.get("CAM", "saturation"),
                                      save_raw      = config.get("CAM", "raw"), 
                                      blocking      = True)
            
            config.imglist.extend(imgpaths)

            # rotate stepper to next photo angle
            stepper.move_to_angle((360/IMGCOUNT) * (i+1))
            sleep(0.5)
        
        stepper.move_to_angle(0)
        stepper.move_steps(1)
        sleep(0.5)


    # 180° SCAN
    if enable_lidar:
        assert lidar is not None, "Lidar must be initialized"
        print("\nLIDAR STARTED...\n")
        lidar.read_loop(callback=move_steps_callback, max_packages=config.max_packages)
        
        stepper.move_to_angle(0)   # return to 0°

        # Power off stepper motor to prevent overheating
        if hasattr(config, 'relay_device') and config.relay_device is not None:
            config.relay_device.off()
            print("Stepper motor powered off.")

        # Save raw_scan to pickle file
        raw_scan = get_scan_dict(lidar.z_angles, cartesian_list=lidar.cartesian_list)

        if enable_IMU:
            # inject IMU data into raw_scan dict
            raw_scan["quaternions"] = quat_list

        save_raw_scan(lidar.raw_path, raw_scan)
    

finally:
    print("\nPiLiDAR STOPPED\n")
    
    # Power off stepper motor before closing
    if hasattr(config, 'relay_device') and config.relay_device is not None:
        config.relay_device.off()
    
    if enable_lidar and lidar is not None:
        lidar.close()

    if enable_IMU and imu is not None:
        imu.close()

    stepper.close()
    
    if hasattr(config, 'relay_device'):
        config.close_gpio()


# STITCHING PROCESS
if enable_cam:
    print("\nStitching Pano...")
    project_path = hugin_stitch(config)


# 3D PROCESSING
if config.get("ENABLE_3D"):
    print("\nProcessing 3D Point Cloud...")
    # Force NumPy backend on Raspberry Pi 5 for stability and ASCII PLY output
    _ = process_raw_numpy(config, save=True)
