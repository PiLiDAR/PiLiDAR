# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.6.7] - 2026-10-14

### Fixed
- **On-Pi 3D processing OOM-killed for large scans.** A 360° sweep (~4.3 M points) exhausted the 4 GB Pi during `process_raw` (normal estimation + open3d). `PiLiDAR.py` now skips the on-Pi 3D export when the point count exceeds `3D.MAX_POINTS_ON_PI` (default 3 000 000) and tells you to run `pilidar process <scan_id>` on a desktop — the raw container + panorama are already saved, so nothing is lost. No-op off the Pi / under the threshold.

## [2.6.6] - 2026-10-14

### Added
- **`experiments/calibrate_extrinsics.py`** — LiDAR↔camera panoramic extrinsic calibration. Renders the intensity LiDAR panorama under a parameterised projection (projection **centre** + yaw/pitch/roll) and maximises the masked zero-mean NCC of gradient maps against the stitched colour panorama (multi-scale coarse→fine, Nelder-Mead). Writes an overlay (red=colour edges, green=LiDAR edges, yellow=match) for visual verification. This recovers the parameters that then drive vertex colouring.

### Notes
- The 360° sweep (`STEPPER.SCAN_ANGLE=360`) produces ~4.3 M points; `process_raw` **OOM-kills on the 4 GB Pi** (acquisition + pano are fine — only the on-Pi 3D step). Processing must run on the desktop (or on Pi with filtering/lower memory).

## [2.6.5] - 2026-10-14

### Changed
- **LiDAR USB power is deferred until the scan actually starts.** Previously `PiLiDAR.py` powered the LiDAR on at launch, so it spun (and drained the battery) through the whole photo/HDR phase. Now the power-on — and the `Lidar` construction + serial-port resolution that needs it (the port only enumerates while powered) — happens immediately before the read loop. `atexit` still guarantees power-off on any exit. No-op off the Pi.
  - Side effect: the photo loop no longer touches `lidar.z_angle` (it never needed the LiDAR); it reads the stepper angle directly.

## [2.6.4] - 2026-10-12

### Added
- **Projection centre for the panoramic projection.** `angular_from_cartesian(points, center=...)` now subtracts an explicit origin, so the equirectangular render can be done about any point (the effective camera nodal point) — the principal knob for LiDAR↔camera extrinsic calibration.
- **`get_lidar_pano(pcd, W, H, intensity=..., center=..., z_rotate=...)` now uses *true* laser intensity** (falls back to the colour red channel only if not supplied) and is vectorised via the new `render_lidar_pano()`. Previously it read `colors[:,0]` from the *colourised* cloud, which made any alignment against the panorama circular.
- Tests: centre = origin-shift, intensity coercion (0–1 / 0–255), and a pano render smoke test. Suite now 81 tests.

### Notes
- Raw intensity is available in the merged array (`merge_2D_points` col 3); it is not currently written to the PLY, so the LiDAR panorama must be built from the raw scan. Preserving intensity through export is a follow-up.

## [2.6.3] - 2026-10-12

### Added
- **Scan provenance in the raw-scan sidecar.** `get_scan_dict()` now accepts a `header`, and `PiLiDAR.py` fills it with `lib.file_utils.scan_provenance(config)`: scan id, `created_utc`, hostname (`author`), `platform`, `hardware` (device-tree model), `software_version`, `device_id`, and a config snapshot (`sampling_rate`, `scan_angle_deg`, `target_res_deg`, `h_res_deg`, `imu_leveling`). The header fields were previously all `null`.
- **`lib/version.py`** — single source of truth for the software version, read from `version.txt`; the CLI and provenance both use it (no more hardcoded CLI version).
- **`lib.platform_utils.get_hardware_model()`** — best-effort device-tree model string for provenance.
- Tests for custom headers and provenance. Suite now 78 tests.

## [2.6.2] - 2026-10-12

### Fixed
- **Raw-scan sidecar was lying about the data.** `_write_metadata()` wrote a **hardcoded** array schema that did not match what was saved: it claimed `quaternions: int16 [n_planes, 4]` while the `.npz` actually held `int64 [n_planes+1, 4]`. The schema is now derived from the **actual** arrays (`<dtype> [<shape>]`), so it can never drift. Quaternions are now stored as **int16** (the DMP's native type; `np.asarray` had promoted the Python ints to int64).
- Added `n_quaternions` and `warmup_quaternion_extra` (= `len(quaternions) - n_planes`, i.e. the dropped warm-up plane's extra leading sample) to the sidecar, so the off-by-one is explicit rather than surprising.
- Regenerated the local scans' containers; test now asserts the sidecar schema matches the arrays.

## [2.6.1] - 2026-10-12

### Added
- **Unified `pilidar` CLI** (`lib/cli.py`, `pilidar.py`, console script) — the first piece of Phase 3. One entry point for the previously scattered scripts:
  - `pilidar doctor` — pre-flight hardware diagnostics (wired to `lib.diagnostics`).
  - `pilidar process <scan_id>` — assemble + colour + level a raw scan (with `--no-save` / `--no-level` / `--no-colour`).
  - `pilidar convert [scan_ids]` — legacy `.pkl` → open NPZ+JSON container.
  - `pilidar export <in> [out]` — point-cloud format conversion (PLY/PCD/…).
  - `pilidar capture` / `pilidar register` — reserved, give clear "not yet wired" errors.
- `lib.pointcloud.convert_raw_scan()` shared helper (used by both the CLI and the experiment).
- Tests for the CLI (sub-parser wiring, version/help, convert round-trip). Suite now 76 tests.

## [2.6.0] - 2026-10-12

### Changed
- **Raw scans now use an open container instead of pickle.** `save_raw_scan()` writes `<scan_id>_lidar.npz` (arrays) + `<scan_id>_lidar.json` (metadata sidecar: header, plane/point counts, angle range, `has_quaternions`) — no more Python-locked `.pkl`. The ragged per-plane fans are stacked losslessly into one `(n_points, cols+1)` array (column 0 = plane index) so the per-plane point counts survive.
- **`load_raw_scan()` prefers the open container** and transparently falls back to the legacy `.pkl`, so existing scans keep loading unchanged. `process_raw()` detects either form.
- `PiLiDAR.py` / `lib/lidar_driver.py` need no change — they call `save_raw_scan`; new scans simply emit the open container.

### Added
- **`experiments/convert_scans.py`:** convert legacy `*_lidar.pkl` scans to the open container (leaves the `.pkl` in place). Validated: lossless round-trip on real scan `261011-2329` (983 planes, 2.1 M points, quaternions).
- Tests: open-container round-trip, no-quaternions, and legacy-pickle fallback. Suite now 71 tests.

## [2.5.14] - 2026-10-11

### Fixed
- **MPU6050 DMP init retries widened 5 → 10.** The flaky I2C `EIO`/`EREMOTEIO` DMP upload can need more than 5 attempts (observed ~3–6, occasionally more); 5 still let scans capture without quaternions (`261011-2345`). 10 retries captured quaternions on `261011-2354`.

### Notes
- **Cross-scan mounting consistency is ~2°.** `IMU.MOUNTING` solved on the tilted `261011-2329` (18.4°) levels it to 0.10°, but on the level `261011-2354` (2.2°) the residual is ~1.6° — the body-frame turntable axis (`fit_turntable_axis.c_body_accel`) differs ~2° between the two scans (spread ~14°), so the calibrated mounting does not fully generalise. Improving the axis estimate (or a joint multi-scan solve) is the next accuracy step.

## [2.5.13] - 2026-10-11

### Fixed
- **`reference_gravity()` crashed when the sweep has one more quaternion than slices.** The dropped warm-up plane is still sampled by the IMU, so `quaternions` leads `z_angles` by one (and the extra leading sample is at the stale pre-sweep attitude). `reference_gravity()` now drops the leading excess sample before indexing, instead of raising `IndexError: boolean index did not match ...`. This aborted the 3D step (no `.ply`) on the first scan captured with quaternions + the warm-up fix (`261011-2329`).

### Changed
- **Re-solved `IMU.MOUNTING` on a tilted rig** (`261011-2329`, 18.4° tilt) with `experiments/calibrate_mounting.py`. The previous matrix was fit to the *old biased mean gravity* (~89° off the corrected gravity); the new one is consistent with `reference_gravity` (floor tilt after leveling **0.10°**, down from the earlier partial `6°`/`4.2°`).

## [2.5.12] - 2026-10-11

### Fixed
- **Flaky MPU6050 DMP initialisation silently disabled the IMU.** The DMP firmware upload intermittently fails with an I2C `EIO`/`EREMOTEIO` even though the chip answers at its address (`i2c` scan shows `0x68`); `MPU6050Wrapper` now retries the init up to 5×. Previously a single glitch made `PiLiDAR.py` drop the IMU for the whole scan, so the pickle had no `quaternions` and leveling could not run (seen on `261011-2213`, `261011-2301`, `261011-2320`).

## [2.5.11] - 2026-10-11

### Fixed
- **Broken first plane of every sweep (stale `z_angle` + partial arc).** The serial stream is live and the turntable already turning when a scan starts, so the first plane spans *stream-start → first anchor crossing*: a **partial** fan labelled with the **stale** pre-sweep `z_angle` (e.g. `269.99°` before a `0.19°` sweep start). Revolved by `-z_angle` it landed ~90–270° from its true azimuth (the spurious tilted slice visible in every scan). `Lidar._finalize_plane()` now drops the first (warm-up) plane in anchor mode — the sweep begins cleanly at the first crossing. Legacy fixed-frame mode is unchanged.
- **IMU leveling under-corrected a tilted scan (~60 %).** `imu_up_vector()` averaged gravity over the sweep, but over a 180° sweep gravity traces only a **half-cone** about the turntable axis, so the normalized mean lies *inside* the cone and under-reported the tilt. New **`reference_gravity()`** samples gravity at a single attitude (stepper angle ≈ 0°) and is not cone-biased; `process_raw()` now levels with it.

### Added
- **`reference_gravity()`** and **`solve_mounting()`** (`lib/imu_orientation.py`): gravity at the reference attitude, and a least-squares IMU→cloud mounting solver from two vector correspondences (turntable axis → +Z, reference gravity → floor normal).
- **`experiments/calibrate_mounting.py`**: solve and validate `IMU.MOUNTING` from a scan (needs quaternions; use a clearly **tilted** rig so the two constraints are not degenerate).
- **`experiments/floor_tilt.py`**: RANSAC floor-plane tilt metric (raw vs leveled) — the honest "is the floor horizontal?" test.
- Tests: `reference_gravity` (recovers true gravity where the mean is biased; levels a synthetic tilted floor), `solve_mounting` (maps axis→+Z and gravity→floor, proper rotation), and the acquisition warm-up drop. Suite now 67 tests.

### Notes
- ⚠️ **`IMU.MOUNTING` must be re-solved** (`experiments/calibrate_mounting.py`) on a tilted scan before the reference-gravity leveling is trustworthy: the matrix currently in `config.json` was calibrated to map the *old biased* mean gravity to the floor, so it is ~89° off in azimuth relative to the corrected gravity (the old result only "worked" by the bias cancelling that error). Until re-calibration, leveled exports may regress.

## [2.5.10] - 2026-10-11

### Added
- **IMU gravity leveling (`lib/pointcloud.py`, `lib/imu_orientation.py`):** the assembled point cloud is now leveled to gravity using the scan's IMU quaternions. The scanner up is the measured gravity direction in the IMU frame, mapped into the cloud frame by the mounting (`_IMU_TO_CLOUD`: X=left, Y=up, Z=forward; the LiDAR's local up is −Z, so the scan plane is the IMU X–Y plane) via `cloud_up_vector()`, then `leveling_rotation()` produces the single rotation applied to the finished cloud.
  - **Applied once, at the end** (in `postprocess_3D`, after vertex colouring and before saving) — assembling in device space and rotating the result is mathematically identical to rotating every plane, and the panorama colour lookup is referenced to the device frame, so colouring must happen before leveling. Gated by `IMU.LEVEL` (default true).
- **`tests/test_imu_axis.py`:** `cloud_up_vector` test (IMU +Y → cloud +Z, proper rotation) — suite now 60 tests.

### Verified
- On real scan `261011-1953` (984 quaternions): measured up in the cloud frame `[-0.037, -0.033, +0.999]` (2.82° tilt) → after leveling `[0, 0, 1]`. IMU up = +Y confirmed, matching the stated mounting.

## [2.5.9] - 2026-10-11

### Fixed
- **MPU6050 quaternion corruption (bit-banged I²C + slow FIFO drain):** the IMU sat on `i2c-gpio` (software I²C on GPIO22/27, because hardware I²C-1 needs GPIO3 = the power button). The `mpu6050` library drains the DMP FIFO **one byte per I²C transaction**, which the bit-banged bus cannot sustain at 50 Hz × 42 B — the FIFO fills to 1024 and the byte stream loses alignment, so the saved quaternions were garbage (e.g. norms 25000–55000 instead of ~16384). `lib/imu_driver.py` now reads the FIFO with **bulk block transactions** (≤32 B, within the SMBus limit) and **validates the quaternion norm**, resetting the FIFO to re-sync on any mismatch.

### Changed
- **IMU moved to a hardware I²C bus:** `dtoverlay=i2c-gpio,...bus=3...` → `dtoverlay=i2c6,pins_22_23` (hardware `bcm2835` bus; SDA stays GPIO22/pin 15, SCL moves GPIO27/pin 13 → GPIO23/pin 16). `IMU.i2c_bus` 3 → 6; `docs/setup.md` documents the new overlay, pins and the hardware-vs-bit-banged check.

### Added
- **`fit_turntable_axis()` (`lib/imu_orientation.py`):** fits the turntable up-axis from a sweep's IMU quaternions two ways — the common axis of the incremental rotations (body and gravity-referenced world frames; drift-averaged over the sweep) and an independent gravity-cone plane fit — plus the rig tilt. `tests/test_imu_axis.py` (4 tests) validates both on synthetic data; suite now 57 tests.
- **`experiments/up_vector_probe.py`:** logs the IMU quaternion at every stepper angle over a 360° sweep to `/tmp/up_vector_sweep.csv` (raw values, so corruption is visible).

## [2.5.8] - 2026-10-11

### Fixed
- **HDR camera regression (the exposure calibration was inverted since the `rpicam‑still` → Picamera2 rewrite):** the old code derived `exposure = auto_exposure × auto_gain / set_gain`; the rewrite had the ratio **flipped**, so manual photos came out nearly black and the bracket collapsed. Now re-expressed around a scene **illumination value** (`illumination = exposure × gain`) with a clean transform back to `(ISO, shutter)`.

### Changed
- **EV-based exposure model (`lib/picam2_utils.py`):** `illumination()`, `ev_transform()` and `bracket_exposures()` replace the ad-hoc gain helpers. `estimate_parameters()` returns the measured illumination; `take_hdr()` builds the bracket `[-AEB·stops … +AEB·stops]` from it.
- **Fixed gain per bracket pass, ISO from config:** `CAM.gain_low` (ISO 100) for steps ≤ 0 and `CAM.gain_high` (ISO 200) for steps > 0 — so dark steps lengthen shutter, bright steps raise gain, and ISO never exceeds 200. `CAM.max_exposure`, `base_gain` and `max_gain` removed (obsolete).
- **`CAM.preview_dims` → `CAM.calibration_dims`** (it is now only the 3A calibration resolution), and the internal `configure(..., lores_dims=...)` parameter renamed to match.
- **`_capture_at()` uses a proportional exposure tolerance** (no absolute 1 ms floor) and verifies both `ExposureTime` and `AnalogueGain`, so every saved frame genuinely carries the requested setting — the 1 ms floor was silently accepting the previous (brighter) frame for the sub-millisecond dark steps.

### Verified
- **On hardware (Pi + IMX477):** auto-exposed `ev` re-transformed gives ISO 100 @ 0.80 ms reproducing the auto frame (lum 130.7 vs 128.2); the −2…+2 bracket is monotonic and evenly spaced (28.8 / 83.6 / 130.8 / 182.1 / 223.3). Test script: `experiments/test_picam2_exposure.py` (writes to `/tmp/picam2_exp/`).

## [2.5.7] - 2026-10-11

### Changed
- **Stepper step is now phase-locked to the LiDAR beam angle (`lib/lidar_driver.py`):** the turntable used to step on a blind 180-packet frame boundary, so the seam landed at an arbitrary LiDAR angle. New `LIDAR.<DEVICE>.STEP_AT_ANGLE` (default 90°) makes the driver step when the beam *crosses* that angle — the point nearest the rotation axis, where the platform's rotation causes the least motion smear. Crossings are detected by edge detection (one step per LiDAR revolution); in anchor mode the crossing is the **only** plane boundary (a fixed packet frame would beat against it and cut planes short) with a `plane_max` safety cap so a missed crossing cannot overflow the preallocated arrays. Verified on hardware: steps fire consistently at FSA ≈ 90.5°, one per revolution, uniform 181-packet planes.
  - Because the LiDAR is mounted *off* the rotation axis (it sits back-to-back with the camera, offset in height/distance), the minimum-radius angle is not exactly 90°/270° — tune `STEP_AT_ANGLE` on the rig. Either pole works (both lie on the axis).

### Fixed
- **Plane/fan off-by-one (`lib/lidar_driver.py`):** `read_loop` called the stepper callback (which advances `z_angle`) *before* appending `self.z_angle`, so every fan was labelled with the **next** angle while it was captured at the previous one — a systematic one-step (~0.18°) rotation of the whole cloud. The plane is now finalised (and labelled with its captured angle) before the step, via the extracted `_finalize_plane()`.

### Added
- **`tests/test_packet.py`:** 8 tests for the anchor crossing detection, the anchor-mode plane boundary (including the frame-full-must-not-cut and safety-cap cases), and the captured-angle labelling — suite now 48 tests.

## [2.5.6] - 2026-10-11

### Added
- **Scan-button long-press to toggle USB power (`gpio_interrupt.py`):** the button now has two gestures — *short press* starts a scan, *long press (> 2 s)* toggles USB-A hub power (`usb_power.hub_on/hub_off`). This is needed because the LiDAR is USB-powered and the Pi 4 can only switch the whole USB-A hub, so keeping the LiDAR off while idle leaves USB-A unpowered — the long press is the deliberate "give me USB-A" gesture to attach a USB stick for `usb_dump`. Implemented with `BOTH_EDGES` + a `threading.Timer`: the falling edge arms the timer, rising edge cancels it (short press → scan) or it fires (long press → toggle).
- **Low-level `hub_on()`/`hub_off()`/`hub_is_on()` (`lib/usb_power.py`):** plain hub power primitives (no device wait) used by the button gesture; the `usb_power` and `ttyUSB`-renumbering logic is described below.
- **`tests/test_button.py`:** 4 tests for the gesture logic (short → scan, long → toggle, toggle direction) using mocked hardware — suite now 39 tests.

### Changed
- **`gpio_interrupt.py`:** imports `lgpio` lazily inside `main()` so the module is importable (and testable) on desktop; the boot-time USB power cut moved into `main()`.
- **`docs/setup.md`:** scan-button section documents the short/long-press gestures.

## [2.5.5] - 2026-10-11

### Added
- **LiDAR USB power gating (`lib/usb_power.py`):** the STL27L is USB-powered, so its VBUS can be cut while idle (fully off, no spin, no power draw) instead of running from boot — this replaces the obsolete LD06 power relay. `PiLiDAR.py` powers the hub on *before* `Config()` resolves the serial port (otherwise it falls back to `/dev/ttyS0`) and cuts it again right after acquisition, before the (long) stitching/3D post-processing; `gpio_interrupt.py` cuts it at boot and also if a scan process died before its own cleanup. Configured via an `USB_POWER` block in `config.json` (`ENABLE`/`HUB`/`DELAY`). Validated step-by-step on hardware with the unit's power LED as ground truth.
- **Stable serial path resolution:** power-cycling the USB port renumbers the LiDAR (`ttyUSB0` → `ttyUSB1` → …), so the port is now resolved from the stable `/dev/serial/by-id/...` link (`LIDAR.<DEVICE>.PORT_BY_ID` in `config.json`) with the raw `PORT` as fallback — in both `lib/usb_power.py` and `lib/config.py` (`Config.resolve_port()`).
- **`tests/test_usb_power.py`:** 5 tests for the config plumbing (defaults, missing block, by-id resolution, disabled no-op) — suite now 35 tests.

### Changed
- **`docs/setup.md`:** USB power-switching section rewritten with the working Pi 4 method and the caveat below.

### Notes
- **Pi 4 USB power: whole-hub only.** On the Pi 4 the USB-A switch is in the VL805 (bus 2) and *only whole-hub* switching actually cuts power: `sudo uhubctl -l 2 -a 0` (off) / `-a 1` (on). Per-port (`-p`) and exact-location (`-e`) flags only drop the USB *data* link — the LiDAR keeps running (confirmed: blue LED stays on while the CP210x disappears from the bus). The trade-off is that the whole-hub cut powers down **all four USB-A ports** (blue 3.0 and black 2.0 alike), so nothing that must stay powered during the idle window should sit on USB-A. A per-device switch (the freed relay pin GPIO24, or a MOSFET) would remove that limitation once rewired.
- **Motor idle power (A4988 `/EN`):** postponed — the A4988 holding current/holding-lock while idle is a real issue but the fix needs a hardware change (route `/EN` to a free GPIO; the freed relay pin GPIO24 is a candidate) and cannot be done today without making the rig unusable.

### Fixed
- **`Config.gpio_setup()` aborting the scan on a busy GPIO (`lib/config.py`):** `lgpio.gpio_claim_output()` *raises* `lgpio.error` when a pin is already claimed (e.g. a second PiLiDAR instance or a still-running scan), and the old code only caught `ImportError`/`RuntimeError` — so a busy relay pin crashed `Config()` at startup. It now catches `lgpio.error` (best-effort `gpio_write`) and any failure is a warning, never fatal.
- **`lidar_on()` resolved the serial device before power-on (`lib/usb_power.py`):** the stable `/dev/serial/by-id` link only exists while the LiDAR is powered, so resolving it first fell back to `/dev/ttyUSB0` and then timed out (the re-powered device actually appears as `ttyUSB1`). The device is now resolved *after* power-on.

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
