# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.2.0] - 2026-10-11

### Added
- **Native Picamera2 camera backend (`lib/picam2_utils.py`):** Replaces the `rpicam-still` subprocess hack with a single persistent Picamera2 session. Calibration now lets the 3A pipeline settle over a few frames, reads the converged `ExposureTime` / `AnalogueGain` / `DigitalGain` / `ColourGains` from `capture_metadata()`, then locks 3A (`AeEnable=False`, `AwbEnable=False`) and drives the sensor manually — no more per-frame process spawn or EXIF round-trips.
  - **HDR bracket preserved:** keeps the shutter-for-dark / gain-for-bright strategy (negative EV steps lengthen exposure, positive EV steps raise gain) to avoid motion blur on bright frames. Total gain is split into analogue (capped at `MAX_ANALOGUE_GAIN = 16`) and digital components; `DigitalGain` stays at 1 below the cap.
  - **Unit handling:** Picamera2 reports exposure in nanoseconds and gain as linear multipliers; the module converts to the project's seconds / total-gain convention at the boundary.
  - **Drop-in interface:** exposes `estimate_camera_parameters()` and `take_HDR_photo()` matching `lib/rpicam_utils.py`, plus a `Camera` context manager for keeping one session across all angles.
- **Backend auto-selection (`PiLiDAR.py`):** imports Picamera2 when available and falls back to the `rpicam-still` path otherwise (e.g. on a desktop without the camera stack). The chosen backend is logged during calibration.

## [2.1.1] - 2026-10-11

### Fixed
- **Template image-ordering bug (`lib/pano_utils.py`):** `take_HDR_photo()` emits each angle's bracket in capture order `[−4,−2,0,+2,+4]`, but the Hugin templates list images in `[0,+2,−2,+4,−4]` (EV-0 first). Because the templates carry stale Windows paths, `pto_template` matches images *positionally*, so the EV-0 anchor and shared-distortion priors were landing on the wrong (−4 EV) frame. Added `_template_ev_order()` (parses the per-angle EV sequence from a template) and `_reorder_imglist()` (re-sorts each angle group to match it), and wired both into `hugin_stitch()` and `hugin_refine()` before `pto_gen`. Verified on scan `240827-1322`: the first angle group now maps `[0,+2,−2,+4,−4]` as the template expects.

## [2.1.0] - 2026-10-11

### Added
- **Hugin Refine Pipeline (`lib/pano_utils.py`):** Added a feature-matching + constrained-optimization stitching path (`hugin_refine()`) that runs `cpfind`/`cpclean`/`linefind`/`pto_var`/`autooptimiser` before blending, correcting per-mount extrinsics (yaw/pitch/roll) and shared lens distortion instead of trusting the fixed template. Validated on scan `240827-1322`: seam ghosting (e.g. the gray pillows) is eliminated versus the template-only original.
  - **Corrected `pto_var`:** locks translations (no `TrX/TrY/TrZ`, avoiding parallax in a rotational pano), fully anchors image 0 (`!r0,!p0,!y0`), and optimizes distortion on all images so they converge consistently.
  - **Control-point fallback:** if `cpclean` yields fewer than `min_control_points` (default 5), optimization is skipped and the template priors are used as-is (textureless scenes).
- **Flatpak-aware Hugin runner (`lib/pano_utils.py`):** `_run_hugin()` / `_hugin_prefix()` resolve each Hugin CLI tool from PATH first, then fall back to the Flatpak build (`net.sourceforge.Hugin`). Both `hugin_stitch()` and `hugin_refine()` now use it.
- **Experiment Harness (`experiments/refine_experiment.py`):** Standalone script to run the refine pipeline on an existing scan and compare against the original output.

### Changed
- **Refine by default (`PiLiDAR.py`, `config.json`):** The stitching step now calls `hugin_refine()` when `PANO.REFINE` is `true` (default), falling back to the fast template-only `hugin_stitch()` when `false`.
- **Experiment Relocation:** Moved ad-hoc experiment scripts into a dedicated `experiments/` directory.
- **Roadmap Consolidation (`ROADMAP.md`):** Folded the camera 3A/HDR and Hugin template analysis into the Phase 1 (Camera Subsystem Modernization) and Phase 2 (Panorama Geometry Refinement) sections, including the concrete Picamera2 migration notes and the remaining template-ordering / shared-distortion work.

### Removed
- **`MOST-OBVIOUS-ISSUES.md`:** Content consolidated into `ROADMAP.md`; file removed.

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
