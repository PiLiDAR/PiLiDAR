from time import sleep
import os

from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
from lib.imu_driver import MPU6050Wrapper
from lib.config import Config, format_value
from lib.pointcloud import process_raw, save_raw_scan, get_scan_dict
from lib.picam2_utils import Camera
from lib.pano_utils import hugin_stitch, hugin_refine
from lib.platform_utils import get_platform
from lib import usb_power


print(r'''
 ____  _ _     _ ____    _    ____
|  _ \(_) |   (_)  _ \  / \  |  _ \
| |_) | | |   | | | | |/ _ \ | |_) |
|  __/| | |___| | |_| / ___ \|  _ <
|_|   |_|_____|_|____/_/   \_\_| \_\
''')


# Power the LiDAR USB port on BEFORE Config() resolves the serial port: if the
# LiDAR VBUS is cut (idle), the port is absent and Config falls back to /dev/ttyS0.
lidar_usb_powered = False
if get_platform() == 'RaspberryPi':
    lidar_usb_powered = usb_power.lidar_on()

config = Config()
config.init()

enable_cam   = config.get("ENABLE_CAM")
enable_lidar = config.get("ENABLE_LIDAR")
enable_IMU   = config.get("ENABLE_IMU")

# initialize IMU
if enable_IMU:
    imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"), config.get("IMU", "frequency"))
    quat_list = []


# initialize stepper
stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"),
                config.get("STEPPER", "pins", "STEP_PIN"),
                config.get("STEPPER", "pins", "MS_PINS"),
                delay=config.get("STEPPER", "STEP_DELAY"),
                step_angle=config.get("STEPPER", "STEP_ANGLE"),
                microsteps=config.get("STEPPER", "MICROSTEPS"),
                gear_ratio=config.get("STEPPER", "GEAR_RATIO"))


# initialize lidar
if enable_lidar:
    lidar = Lidar(config, visualization=None)

    # callback function for lidar.read_loop()
    def move_steps_callback():
        stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
        lidar.z_angle = stepper.get_current_angle()

        # # DEBUG: take photo at each step
        # imgpath = os.path.join(config.scan_dir, f"{format_value(lidar.z_angle, 2)}.jpg")
        # cam.take_photo(path=imgpath,
        #                exposure_time=current_exposure_time,
        #                gain=current_gain,
        #                awbgains=current_awbgains)

        if enable_IMU:
            # euler = imu.get_euler_angles()
            # print(f'{format_value(lidar.z_angle, 2)} | Euler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}')
            quat_values = imu.get_quat_values()
            quat_list.append(quat_values)  # [lidar.z_angle, *quat_values]

    if not enable_cam:
        # wait for lidar to lock rotational speed
        sleep(2)

# MAIN
try:
    # 360° SHOOTING PHOTOS
    if enable_cam:
        with Camera() as cam:
            # CALIBRATE CAMERA: converge 3A and lock exposure/gain/AWB
            print("Calibrating Camera...")
            current_exposure_time, current_gain, current_awbgains = cam.estimate_parameters(config)
            # print("[RESULT] AE:", current_exposure_time, "| Gain:", current_gain, "| AWB R:", round(current_awbgains[0],3), "B:", round(current_awbgains[1],3))

            # switch to full-resolution stills for the panorama
            cam.configure(config.get("CAM", "dims"))

            IMGCOUNT = config.get("PANO", "IMGCOUNT")
            for i in range(IMGCOUNT):
                lidar.z_angle = stepper.get_current_angle()
                formatted_angle = format_value(lidar.z_angle, config.get("ANGULAR_DIGITS"))

                print(f"\nTaking photo {i+1}/{IMGCOUNT} | Angle: {formatted_angle}")
                imgpath = os.path.join(config.img_dir, f"image_{formatted_angle}.jpg")

                if enable_IMU:
                    euler = imu.get_euler_angles()
                    print(f'\tEuler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}')

                # take HDR photo
                imgpaths = cam.take_hdr(path          = imgpath,
                                        AEB           = config.get("CAM", "AEB"),
                                        AEB_stops     = config.get("CAM", "AEB_STOPS"),
                                        exposure_time = current_exposure_time,
                                        gain          = current_gain,
                                        awbgains      = current_awbgains)

                config.imglist.extend(imgpaths)

                # rotate stepper to next photo angle
                stepper.move_to_angle((360/IMGCOUNT) * (i+1))
                sleep(0.5)

            stepper.move_to_angle(0)
            stepper.move_steps(1)
            sleep(0.5)


    # 180° SCAN
    if enable_lidar:
        print("\nLIDAR STARTED...\n")
        lidar.read_loop(callback=move_steps_callback, max_packages=config.max_packages)

        stepper.move_to_angle(0)   # return to 0°

        # Save raw_scan to pickle file
        raw_scan = get_scan_dict(lidar.z_angles, cartesian_list=lidar.cartesian_list)

        if enable_IMU:
            # inject IMU data into raw_scan dict
            raw_scan["quaternions"] = quat_list

        save_raw_scan(lidar.raw_path, raw_scan)


finally:
    print("\nPiLiDAR STOPPED\n")
    if enable_lidar:
        lidar.close()

        if enable_IMU:
            imu.close()

    stepper.close()

    # Cut LiDAR USB power (fully off while idle). The old LD06 power relay is
    # obsolete: the STL27L is USB-powered, see lib/usb_power.py.
    if lidar_usb_powered:
        usb_power.lidar_off()


# STITCHING PROCESS
if enable_cam:
    # REFINE runs feature matching + optimization (slower, better alignment);
    # otherwise fall back to the fast template-only stitch.
    if config.get("PANO", "REFINE"):
        print("\nStitching Pano (refine: feature matching + optimization)...")
        project_path = hugin_refine(config)
    else:
        print("\nStitching Pano (template only)...")
        project_path = hugin_stitch(config)


# 3D PROCESSING
if config.get("ENABLE_3D"):
    print("\nProcessing 3D Point Cloud...")
    pcd = process_raw(config, save=True)
