# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.5.5] - 2026-10-11

### Added
- **LiDAR USB power gating (`lib/usb_power.py`):** the STL27L is USB-powered behind a per-port-power-switching hub, so its VBUS can be cut while idle (fully off, motor stopped) instead of leaving it powered from boot. `PiLiDAR.py` powers the port on *before* `Config()` resolves the serial port (otherwise it falls back to `/dev/ttyS0`) and cuts it on exit; `gpio_interrupt.py` also cuts it if a scan process died before its own cleanup. Configured via a new `USB_POWER` block in `config.json` (`ENABLE`/`HUB`/`PORT`/`DELAY`). Uses `uhubctl -l <hub> -p <port> -a on/off -e` (the `-e` exact-location flag is required so only the target port switches). Replaces the obsolete LD06 power relay — the LD06 was a bare UART board switched by GPIO, the STL27L has its own USB controller. Verified on hardware: port off → `/dev/ttyUSB0` disappears; on → re-enumerates in ~4 s.
- **`tests/test_usb_power.py`:** 3 tests for the config plumbing (defaults, missing block, disabled no-op) — suite now 33 tests.

### Changed
- **`docs/setup.md`:** the USB power-switching section corrected (the LiDAR is on hub `1-1` port 1, not USB-3 hub 2) and documents the automatic gating + `USB_POWER` config.
- **`PiLiDAR.py`:** replaced the vestigial LD06-relay comments with the USB power-off call; ASCII banner made a raw string (clears escape-sequence warnings).

### Notes
- **Motor idle power (A4988 `/EN`):** postponed — the A4988 holding current/holding-lock while idle is a real issue but the fix needs a hardware change (route `/EN` to a free GPIO; the freed relay pin GPIO24 is a candidate) and cannot be done today without making the rig unusable.

## [2.5.4] - 2026-10-11

### Added
- **Pre-flight hardware diagnostics (`lib/diagnostics.py`):** an automated `pilidar doctor` routine (Phase 1 deliverable) that checks Python version, platform, free storage, Hugin tools, the LiDAR UART packet stream, MPU6050 `WHO_AM_I` over I2C, Picamera2 camera enumeration, and gpiochip access. Hardware checks SKIP cleanly off-Pi, the Hugin check is Flatpak-aware, `--json` gives machine-readable output, and the exit code is 1 on any FAIL (WARN/SKIP non-fatal) so it can gate a scan. Validated on the Pi: 8/8 OK.
- **`tests/test_diagnostics.py`:** 4 tests covering result aggregation, off-Pi hardware SKIP behaviour, the `--skip` argument, and storage reporting (suite now 29 tests).

### Changed
- **`docs/setup.md`:** new "Pre-flight Hardware Checks" section documenting `python -m lib.diagnostics`.

## [2.5.3] - 2026-10-11

### Added
- **Serial throughput benchmark (`experiments/serial_benchmark.py`):** validates the v2.5.0 high-speed serial driver on hardware — runs the background reader + bounded queue at the configured baud rate for a fixed duration and reports packet/sample rate, queue high-water mark, dropped and malformed packet counts against the nominal device rate.

### Verified
- **Zero-drop acquisition on hardware:** STL27L @ 921,600 baud sustained for 30 s on the Pi: 1796.7 packets/s (0.2% deviation from the nominal 1800), 21,561 samples/s, **0 dropped packets, 0 malformed**, queue high-water 340/1024. Completes the Phase 1 "High-Speed Serial Driver Overhaul" benchmark task.

## [2.5.2] - 2026-10-11

### Fixed
- **MPU6050 IMU FIFO overflow during scans (`lib/imu_driver.py`):** the reader thread consumed one DMP packet per 20 ms cycle while the DMP produces at a fixed 50 Hz, so a backlog grew until the chip's 1024-byte FIFO overflowed — visible as periodic `overflow!` prints (from the `mpu6050` library's raw `print`) at a fixed interval, especially during the LiDAR sweep when the serial reader starves the IMU thread of GIL time. Each overflow also reset the FIFO, dropping quaternion samples from `raw_scan["quaternions"]`. The thread now drains the FIFO completely on every wake-up (by FIFO byte count, keeping the freshest quaternion), turning unbounded backlog into ≤20 ms latency. Validated on hardware: 20 s run under 2-thread CPU/GIL pressure with zero overflows (previously dozens).

## [2.5.1] - 2026-10-11

### Fixed
- **`lib/a4988_driver.py` migrated from `RPi.GPIO` to the `lgpio` C-API:** v2.5.0 removed the system `python3-rpi.gpio` package but the stepper driver still imported it, so `PiLiDAR.py` failed at startup on the Pi (`ModuleNotFoundError: No module named 'RPi'`). The driver now claims its pins from gpiochip 0 (`gpiochip_open` → `gpio_claim_output` → `gpio_write`, `gpio_free` on `close()`) — same pattern as `gpio_interrupt.py`, same public API and motion semantics. Validated on hardware: 20° CW/CCW round-trip returns to exactly 0 steps; full `PiLiDAR.py` startup including scan-phase motion works.
- **Relay GPIO setup in `lib/config.py` moved to lgpio** as well: `gpio_setup()` claims `STEPPER.RELAY_PIN` as an output (with a cached chip handle so `Config()` re-inits don't collide). Previously it silently failed with a warning since the `RPi.GPIO` import was gone.

### Changed
- **`docs/troubleshooting.md`:** the RPi.GPIO section no longer recommends installing the `rpi-lgpio` shim — PiLiDAR is now fully on the lgpio C-API; the shim is only mentioned as a fallback for legacy third-party scripts.

## [2.5.0] - 2026-10-11

### Added
- **High-speed serial driver (`lib/lidar_driver.py`):** Replaced byte-by-byte `read()` with buffered block reads (4 KB chunks) and a sliding-window frame parser with automated sync-byte recovery, CRC8 validation, and malformed-packet counting/logging. Added a dedicated background reader thread feeding a bounded thread-safe queue (`LIDAR.<DEVICE>.QUEUE_SIZE`, default 1024) to decouple serial I/O from processing; `read_loop()` now consumes from the queue via `read_from_queue()`.
- **`pymeshlab>=2025.7` dependency:** headless meshing (Poisson surface reconstruction, UV projection / texture baking, Hausdorff cleanup) now runs through the pymeshlab Python API in the project venv — wheels exist for both x86_64 and aarch64, so the same `~/venv` works on the dev PC and the Pi. Requires the Mesa OpenGL runtime (`sudo apt-get install -y libopengl0 libgl1 libegl1`).

### Changed
- **Dependency stack updated to current releases (`pyproject.toml`, `requirements.txt`):** `numpy>=2.0`, `opencv-python>=5.0`, `open3d>=0.20`, `scipy>=1.15`, `matplotlib>=3.10`, `plotly>=6.0`.
- **`jupyter` moved to the `dev` optional-dependency group** — it is only needed for development/visualization, not for production scans.
- **Pi environment standardised on `venv`:** the Pi now uses a plain `python3 -m venv ~/venv` (system Python 3.11 on Bookworm, matching the OS picamera2/libcamera stack) instead of a miniforge3 conda env, matching the desktop dev setup. The system `picamera2`/`libcamera` bridge is provided by a `.pth` file pointing at `/usr/lib/python3/dist-packages` (replaces the old `PYTHONPATH=` prefix).
- **`gpio_interrupt.py` rewritten on the `lgpio` C-API:** replaced deprecated `RPi.GPIO` and the 0.2 s polling loop with `gpiochip_open()` + `gpio_claim_alert()` (falling edge, internal pull-up) + `gpio_set_debounce_micros()` + a callback thread; the main thread now just sleeps — idle CPU drops from ~2% to ~0%. Scan launches now use `~/venv/bin/python` instead of conda.
- **`pi-hardware` GPIO dependency corrected to `lgpio>=0.2.2`** (the C-API module) — the previously listed `rpi-lgpio` is only an RPi.GPIO compatibility shim and does not provide the edge-detection API used by `gpio_interrupt.py`.
- **Pi venv: `simplejpeg` installed in-venv** — the system `simplejpeg` (built against numpy 1.x) is binary-incompatible with numpy 2, so a numpy-2 build is installed into `~/venv` to shadow it for `picamera2`.
- **Documentation restructured into `docs/`:** `ROADMAP.md` moved to `docs/roadmap.md`; new `docs/setup.md` (Python venv, GPIO/UART/PWM, Hugin, MeshLab, USB power), `docs/hardware.md`, `docs/protocol.md`, `docs/troubleshooting.md`, and `docs/references.md` (references split out of troubleshooting). `README.md` rewritten as a concise overview with a Quickstart and a documentation index table. The scan-button systemd service now uses the venv interpreter.

### Removed
- **miniforge3/conda on the Pi:** no longer needed — the plain venv covers all dependencies.
- **Standalone MeshLab installs (flatpak GUI on the dev PC, upstream 2025.07 archive on the Pi, apt `meshlab` 2020.09):** superseded by `pymeshlab` in the venv. The `meshlab/meshlabserver_test.sh` + `.mlx` workflow is kept for reference only.

## [2.4.0] - 2026-10-11

### Added
- **Modern packaging (`pyproject.toml`):** Replaces the unpinned `requirements.txt` as the source of truth for metadata and dependencies. Defines optional dependency groups: `pi-hardware` (GPIO/camera) and `dev` (pytest, ruff). Configures `pytest` and `ruff` (`E`/`F`/`W` rules).
- **Hardware mocking & packet-replay harness (`tests/mock_hardware.py`):** `MockLidarSerial` (a `serial.Serial`-compatible in-memory reader), `build_packet()`/`crc8()` to synthesise valid LiDAR packets from the project's CRC table, plus `MockStepper` and `MockCamera` stand-ins. Lets the acquisition path run on desktop/CI with no GPIO, camera, or serial hardware.
- **Synthetic test fixtures (`tests/fixtures/`):** A checked-in anonymised scan (`synthetic_lidar.pkl`), a matching `lidar/` slice directory, and a raw byte-stream `capture.bin` for replay. Regenerate with `python tests/make_fixture.py`.
- **`pytest` suite (`tests/`, 25 tests):** Covers CRC8 validation and corrupted-packet recovery, the packet `decode()` path (including the LSA<FSA angle wraparound), `read()` byte-stream resync, polar-to-Cartesian transforms, point-cloud merge/NaN removal, `pcd_from_np()` 2D/3D assembly, and fixture loading.
- **GitHub Actions CI (`.github/workflows/ci.yml`):** Runs `ruff check` and `pytest` across Linux, macOS, and Windows on Python 3.11 and 3.12 — no physical hardware required.

### Changed
- **`lib/*` formatting:** Applied `ruff` safe fixes (whitespace, unused imports) across the library. Bare `except:` in import-fallback blocks are intentionally retained (`E722` ignored in config).

## [2.3.0] - 2026-10-11

### Removed
- **`rpicam-still` backend (`lib/rpicam_utils.py`):** Deleted. The native Picamera2 path is now the only camera backend (the project runs on the Pi, where the libcamera stack is always present). This also drops the `exifread` dependency and the rpicam-only `CAM` config keys (`denoise`, `preview_denoise`, `sharpness`, `saturation`, `awbgains`, `max_iterations`, `awb_thres`, `raw`, `remove_tmp`).
- **Module-level wrappers (`lib/picam2_utils.py`):** Removed `estimate_camera_parameters()` / `take_photo()` / `take_HDR_photo()` — each opened and closed its own session, which defeated the point of a persistent session. The `Camera` context manager is now the only API.

### Changed
- **Single persistent-session API (`lib/picam2_utils.py`):** `Camera` now exposes `configure(still_dims, preview_dims=None)` (public), `estimate_parameters(config)`, `take_photo(path, exposure_time, gain, awbgains)`, and `take_hdr(path, AEB, AEB_stops, exposure_time, gain, awbgains)`. Capture methods assume the camera is already configured; `take_hdr()` locks 3A once and flushes the startup auto-mode frame once per bracket.
- **`PiLiDAR.py`:** Camera section restructured to one `with Camera() as cam:` block — calibrate at preview resolution, reconfigure to full-resolution stills, then bracket at each angle. No backend selection or fallback.

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
