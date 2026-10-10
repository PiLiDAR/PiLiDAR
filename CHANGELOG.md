# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] - 2026-10-10

### Added
- **Modern Desktop Tools:** Documented Hugin 2025.0.1 installation via Flathub and MeshLab 2025.07 installation via Flathub on x86_64 or the upstream ARM64 release archive on Raspberry Pi. Kept OS-packaged `hugin-tools` and Enblend documented for the `hugin_executor` pipeline.

### Changed
- **Point Cloud Assembly Optimization (`lib/pointcloud.py`):** Replaced quadratic $O(N^2)$ `np.append` reallocations in `merge_2D_points` with list chunk accumulation and single `np.vstack()`, removing the artificial initial zero row and improving merge speed.
- **Safe Formula Evaluation (`lib/config.py`):** Replaced arbitrary `eval()` with an AST-based mathematical evaluator (`safe_eval_math()`) for stepper and lidar gear/resolution formulas.
- **Robust Platform Detection (`lib/platform_utils.py`):** Switched platform detection from `aarch64` architecture matching to inspecting device-tree models (`/proc/device-tree/model`) and `/etc/rpi-issue`, preventing false positives on desktop ARM64 environments while properly detecting 32-bit Pi OS.
- **Slice Directory Fallback (`lib/pointcloud.py`):** Enabled loading and merging directly from raw `.npy` slice directories (`lidar/`) when a unified pickle file is not present.
- **Lazy GPIO Import (`lib/config.py`):** Replaced top-level unconditional `RPi.GPIO` import with guarded initialization to allow importing configuration on desktop and CI testing environments.

### Added
- **NamedTuple Fallback for Quaternions (`lib/imu_orientation.py`):** Added a lightweight standard library `namedtuple` fallback for `XYZVector` and `Quaternion`, removing the mandatory dependency on the unmaintained `quat` package.
- **Serial Sync Recovery (`lib/lidar_driver.py`):** Added re-checking for consecutive start bytes (`0x54`) to prevent dropped packets when stream desynchronization occurs.
- **Thread Safety for IMU (`lib/imu_driver.py`):** Added a `threading.Lock()` protecting quaternion reads/writes and added null guards to prevent `NoneType` crashes before the first packet is decoded.
- **Clean Worker Shutdown (`lib/imu_driver.py`):** Added thread joining on `close()` for `MPU6050Wrapper`.
- **Version Tracking:** Added `version.txt` and `CHANGELOG.md` following Semantic Versioning.

### Fixed
- **`np.column_stack` Syntax Error (`lib/pointcloud.py`):** Fixed runtime `TypeError` in `load_pointcloud()` by passing arrays as a tuple argument rather than separate arguments.
- **IMU Thread Busy-Spin (`lib/imu_driver.py`):** Fixed 100% CPU core utilization in `_read_mpu6050()` by sleeping when the FIFO buffer is not yet ready.
- **Buffer Index Tracking (`lib/lidar_driver.py`):** `read()` now reports packet status to `read_loop()`, ensuring buffer indices do not advance when incomplete or corrupt CRC packets are encountered.

### Removed
- **Dead Code (`lib/lidar_driver.py`):** Removed orphaned `self.pwm_coeffs` statement in `Lidar.pwm_from_speed()`.
- **Duplicate Normal Calculation (`lib/registration.py`):** Removed redundant `source_down` normal estimation calculation in `global_registration()`.

---

## [1.0.0] - 2024-09-15

### Added
- Initial prototype release of PiLiDAR: 360° LiDAR scanner and panoramic 3D reconstruction system.
- LDRobot LD06 and Waveshare STL27L serial acquisition driver with hardware PWM speed control.
- A4988 stepper motor control with microstepping for rotational scanning.
- MPU6050 IMU driver via DMP FIFO reading for orientation measurement.
- Raspberry Pi HQ Camera (IMX477) capture with AEB bracket exposure control and Hugin panoramic stitching.
- Point cloud generation, transformation, outlier filtering, and mesh reconstruction via Open3D.
- Multi-station registration pipeline (RANSAC, Point-to-Plane ICP, Colored ICP).
- PLY and ASTM E57 point cloud export support.
