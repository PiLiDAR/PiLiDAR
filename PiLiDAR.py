import atexit
import signal
from time import sleep
import os

from lib.lidar_driver import Lidar
from lib.a4988_driver import A4988
from lib.imu_driver import MPU6050Wrapper
from lib.config import Config, format_value
from lib.pointcloud import process_raw, save_raw_scan, get_scan_dict
from lib.file_utils import scan_provenance
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


# The service (gpio_interrupt.py) cuts the LiDAR USB power at boot. The LiDAR is
# powered on only when its read loop is about to start (see MAIN) so its power
# draw does not drain the battery during the photo phase. atexit is the safety
# net for a crash and SIGTERM is converted to SystemExit so the finally/atexit
# handlers still run.
def _on_terminate(signum, frame):
    raise SystemExit(128 + signum)


on_pi = get_platform() == 'RaspberryPi'
usb_power_enabled = usb_power.load_settings()["enabled"]
if on_pi and usb_power_enabled:
    atexit.register(usb_power.lidar_off)
    signal.signal(signal.SIGTERM, _on_terminate)

config = Config()
config.init()

enable_cam   = config.get("ENABLE_CAM")
enable_lidar = config.get("ENABLE_LIDAR")
enable_IMU   = config.get("ENABLE_IMU")

# initialize IMU (non-fatal: a wedged/absent IMU must not abort the scan)
quat_list = []
if enable_IMU:
    try:
        imu = MPU6050Wrapper(config.get("IMU", "i2c_bus"), config.get("IMU", "device_address"), config.get("IMU", "frequency"))
    except Exception as e:
        print(f"[WARNING] IMU init failed, continuing without IMU: {e.__class__.__name__}: {e}")
        enable_IMU = False


# initialize stepper
stepper = A4988(config.get("STEPPER", "pins", "DIR_PIN"),
                config.get("STEPPER", "pins", "STEP_PIN"),
                config.get("STEPPER", "pins", "MS_PINS"),
                delay=config.get("STEPPER", "STEP_DELAY"),
                step_angle=config.get("STEPPER", "STEP_ANGLE"),
                microsteps=config.get("STEPPER", "MICROSTEPS"),
                gear_ratio=config.get("STEPPER", "GEAR_RATIO"))


# the LiDAR is constructed (and USB-powered) later, just before its read loop
# (see MAIN), so its power draw does not drain the battery during the photo phase
lidar = None

# MAIN
try:
    # 360° SHOOTING PHOTOS
    if enable_cam:
        with Camera() as cam:
            # CALIBRATE CAMERA: converge 3A, measure scene illumination, compute manual setting
            print("Calibrating Camera...")
            scene_ev, current_gain, current_exposure_time, current_awbgains = cam.estimate_parameters(config)
            print("[RESULT] EV:", round(scene_ev, 4), "| Gain:", current_gain, "| Exposure:", round(current_exposure_time * 1000, 2), "ms | AWB R:", round(current_awbgains[0], 3), "B:", round(current_awbgains[1], 3))

            # switch to full-resolution stills for the panorama
            cam.configure(config.get("CAM", "dims"))

            IMGCOUNT = config.get("PANO", "IMGCOUNT")
            for i in range(IMGCOUNT):
                photo_angle = stepper.get_current_angle()
                formatted_angle = format_value(photo_angle, config.get("ANGULAR_DIGITS"))

                print(f"\nTaking photo {i+1}/{IMGCOUNT} | Angle: {formatted_angle}")
                imgpath = os.path.join(config.img_dir, f"image_{formatted_angle}.jpg")

                if enable_IMU:
                    euler = imu.get_euler_angles()
                    print(f'\tEuler: x {format_value(euler.x, 2)} y {format_value(euler.y, 2)} z {format_value(euler.z, 2)}')

                # take HDR photo
                imgpaths = cam.take_hdr(path      = imgpath,
                                        ev        = scene_ev,
                                        AEB       = config.get("CAM", "AEB"),
                                        AEB_stops = config.get("CAM", "AEB_STOPS"),
                                        awbgains  = current_awbgains,
                                        gain_low  = config.get("CAM", "gain_low"),
                                        gain_high = config.get("CAM", "gain_high"))

                config.imglist.extend(imgpaths)

                # rotate stepper to next photo angle
                stepper.move_to_angle((360/IMGCOUNT) * (i+1))
                sleep(0.5)

            stepper.move_to_angle(0)
            stepper.move_steps(1)
            sleep(0.5)


    # LIDAR SCAN — power the LiDAR on only now (after the photos) to save battery
    if enable_lidar:
        if on_pi and usb_power_enabled:
            print("\nPowering on LiDAR USB...")
            usb_power.lidar_on()
            config.PORT = config.resolve_port()   # the port enumerates only once powered

        lidar = Lidar(config, visualization=None)

        # callback function for lidar.read_loop()
        def move_steps_callback():
            stepper.move_steps(config.steps if config.SCAN_ANGLE > 0 else -config.steps)
            lidar.z_angle = stepper.get_current_angle()

            if enable_IMU:
                quat_values = imu.get_quat_values()
                quat_list.append(quat_values)  # [lidar.z_angle, *quat_values]

        if not enable_cam:
            # wait for the lidar to lock rotational speed
            sleep(2)

        print("\nLIDAR STARTED...\n")
        lidar.read_loop(callback=move_steps_callback, max_packages=config.max_packages)

        stepper.move_to_angle(0)   # return to 0°

        # Save raw_scan (metadata provenance + arrays) to the open container
        raw_scan = get_scan_dict(lidar.z_angles, cartesian_list=lidar.cartesian_list,
                                 header=scan_provenance(config))

        if enable_IMU:
            # inject IMU data into raw_scan dict
            raw_scan["quaternions"] = quat_list
            # the scanner's IMU body->gravity orientation at stepper 0 (DMP output;
            # rotation axis = body +Y). Store it for later gravity leveling.
            from lib.imu_orientation import scanner_orientation
            q_obj2grav = scanner_orientation(quat_list, lidar.z_angles)
            if q_obj2grav is not None:
                raw_scan["scanner_orientation_xyzw"] = q_obj2grav.as_quat().tolist()

        save_raw_scan(lidar.raw_path, raw_scan)


finally:
    print("\nPiLiDAR STOPPED\n")
    if lidar is not None:
        lidar.close()

    if enable_IMU:
        imu.close()

    stepper.close()

# Acquisition is done: cut the LiDAR USB power now, before the (long) stitching
# and 3D post-processing. atexit (registered above) is the safety net for a crash.
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
    # total_points = sum(len(c) for c in lidar.cartesian_list) if enable_lidar else 0
    print("\nProcessing 3D Point Cloud...")
    pcd = process_raw(config, save=True)
