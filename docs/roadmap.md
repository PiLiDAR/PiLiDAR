# PiLiDAR Project Roadmap

## Repos:
Code: https://github.com/PiLiDAR/PiLiDAR
Hardware: https://github.com/PiLiDAR/PiLiDAR-Hardware

## ACCESS THE DEVICE USING SSH
```bash
ssh pi@192.168.1.16
cd /home/pi/PiLiDAR
```

enable password-less login:
```bash
ssh-copy-id -i ~/.ssh/id_ed25519.pub flip@192.168.1.16
```

## Overview
PiLiDAR is an open-source, low-cost DIY 360° LiDAR scanner and panoramic 3D reconstruction system. This roadmap guides the project from an experimental research prototype into a robust, repeatable, and maintainable platform ready for community use and field deployment.

The project follows a disciplined engineering pipeline:
**Capture -> Validate -> Calibrate -> Process -> Register -> Export**

## Status at a glance
- ✅ **Phase 0** — Foundation, CI/CD, mock harness.
- ✅ **Phase 1** — Hardware abstraction, acquisition stack, diagnostics.
- 🟡 **Phase 2** — Open data standards & calibration (panorama refined; open containers + extrinsics still open).
- ⬜ **Phase 3** — Unified CLI & processing pipeline engine. *(CLI scaffold ✅ v2.6.1)*
- ⬜ **Phase 4** — Field usability: local web UI & remote operation.
- 🟡 **Phase 5** — Multi-scan registration & reconstruction (IMU gravity leveling ✅ done; registration/meshing/E57 open).
- ⬜ **Phase 6** — Packaging, documentation & community release.

## Next up
1. **Phase 2 — LiDAR↔camera 6-DOF extrinsics calibration** (known-target fixture).
2. **Phase 2 — Panorama: shared lens distortion + metadata provenance** in the sidecar.
3. **Phase 3 — finish the CLI:** `capture`/`calibrate`, preprocessing flags (SOR/ROR/voxel), batch mode; retire `process_3D.py`.

---

## Phase 0 — Foundation, CI/CD, and Mock Testing Harness — ✅ done (v2.4.0)
Modern tooling (`pyproject.toml`), hardware mock harness (`MockLidarSerial`/`MockCamera`/`MockStepper` + synthetic fixtures), and a `pytest` suite in GitHub Actions CI (3 OS × Python 3.11/3.12). The suite passes on desktop without any physical hardware.

---

## Phase 1 — Hardware Abstraction & High-Speed Acquisition Stack — ✅ done
- **Serial driver** (v2.5.0/2.5.3): 4 KB block reads + sliding-window CRC8 parser with sync recovery, background reader thread + bounded queue. Benchmarked on hardware: STL27L @ 921,600 baud sustained 1796.7 packets/s, **0 dropped, 0 malformed**.
- **GPIO & motion** (v2.5.1): lgpio C-API throughout (`gpio_interrupt.py`, `lib/a4988_driver.py`, relay); no `RPi.GPIO` imports remain.
- **Camera** (v2.2.0/2.3.0, `lib/picam2_utils.py`): native Picamera2, persistent session, 3A settle→read→lock, shutter-for-dark/gain-for-bright HDR bracket.
- **Diagnostics** (v2.5.4): `python -m lib.diagnostics` (platform/storage/Hugin + UART stream, MPU6050 `WHO_AM_I`, Picamera2, gpiochip). Non-fatal WARN/SKIP, exit 1 on FAIL; validated 8/8 on the Pi.
- **Open (small):** hardware-timer/PWM stepping to remove userspace sleep jitter; homing/endstop to prevent cumulative angle drift.
- **Target OS:** Bookworm (Debian 12); **Trixie (Debian 13) migration planned** — verify lgpio/libcamera/Hugin/pymeshlab and add to the supported matrix.

---

## Phase 2 — Open Data Standards & Geometric Calibration
**Objective:** Replace unversioned pickle storage with open, standardized file formats and establish rigorous calibration procedures.

### Remaining tasks
- **Standardized scan container & metadata** — *open container + provenance ✅ (v2.6.x)*
  - ✅ Raw scans saved as `<id>_lidar.npz` + `<id>_lidar.json` (deprecates pickle writing; `load_raw_scan` falls back to legacy `.pkl`); `pilidar convert` / `experiments/convert_scans.py` convert old scans.
  - ✅ Sidecar carries real provenance: scan id, UTC time, host, platform, hardware model, software version, LiDAR device, config snapshot (sampling/step res/scan angle/IMU leveling) + the array schema.
  - ✅ 3D outputs are already PLY with intensity.
  - ⬜ Move the raw *time-series* (range/angle packets) to a documented schema if needed.
- **LiDAR↔camera 6-DOF extrinsics calibration**
  - Formalize the rigid-body transform ($X, Y, Z, \text{roll}, \text{pitch}, \text{yaw}$) between the LiDAR optical center, the stepper rotation axis, and the camera sensor center.
  - Build a semi-automated calibration utility using a known physical fixture (corner target / planar board).
- **Panorama geometry (remaining)**
  - Make lens distortion a **shared/linked** parameter across all images of the same lens (templates currently bake it into image 0 only, leaving images 1–3 un-corrected).
  - Persist the optimized per-lens distortion + per-mount extrinsics into the scan `metadata.json` so future scans seed from a measured prior instead of a hand-authored template.
  - *Optional* two-stage pipeline: pre-fuse each angle's bracket with `enfuse` into one base image, then solve geometry on the 4 base images (one per 90°).
- **Camera intrinsics & exposure normalization**
  - Automate/document lens-distortion calibration (fisheye / equirectangular projection parameters).
  - Standardize AE/AWB normalization to eliminate seam artifacts in 360° panoramas.

### Done already
- `hugin_refine()` (v2.1.0): `cpfind`/`cpclean`/`linefind`/`pto_var`/`autooptimiser` refine per-mount extrinsics + shared lens distortion from control points (Flatpak-aware, default via `PANO.REFINE`). Validated on `240827-1322` (seam ghosting eliminated).
- `pto_var` corrections: translations locked (rotational pano must not translate), image 0 anchored, distortion optimized across images.
- Template EV-ordering fix (v2.1.1): `_template_ev_order()` + `_reorder_imglist()` align each angle's bracket to the template's EV prior.

### Success criteria
Scans open in third-party tools (CloudCompare, MeshLab, Blender) with no Python/PiLiDAR source required.

---

## Phase 3 — Unified CLI & Processing Pipeline Engine
**Objective:** Replace scattered root scripts with a clean, modular CLI and a deterministic end-to-end processing pipeline.

- **Unified CLI** (`pilidar`, `lib/cli.py`) — *scaffold ✅ (v2.6.1)*:
  - ✅ `pilidar doctor` — pre-flight diagnostics.
  - ✅ `pilidar process <scan_id>` — assemble + colour + level (flags: `--no-save/--no-level/--no-colour`).
  - ✅ `pilidar convert [scan_ids]` — legacy `.pkl` → open NPZ+JSON.
  - ✅ `pilidar export <in> [out]` — PLY/PCD/… conversion.
  - ⬜ `pilidar capture` — acquisition (needs the Pi hardware stack; still `PiLiDAR.py`).
  - ⬜ `pilidar calibrate` — calibration workflows.
- **Preprocessing & noise rejection:** statistical (SOR) and radius (ROR) outlier removal; intensity threshold filtering; voxel downsampling preserving color/intensity. *(exists in `lib/pointcloud.py`; expose as flags.)*
- **Pipeline automation:** progress bars + structured logging; batch processing across raw scan directories.
- Retire ad-hoc scripts (`process_3D.py`); keep `filter_3D.py`/`meshing_test.py` until their CLI equivalents land.

### Success criteria
`pilidar process <scan_dir>` turns a raw capture folder into a clean, vertex-colorized point cloud in a single command.

---

## Phase 4 — Field Usability: Local Web UI & Remote Operation
**Objective:** Make PiLiDAR practical in the field on a tripod without an SSH session or tethered monitor.

- **Lightweight local web controller** (FastAPI/aiohttp), mobile-responsive over the Pi hotspot (`http://pilidar.local`):
  - Real-time state (Idle / Scanning / Stitching / Complete / Error).
  - Trigger scans; presets (Quick Preview / High Res / HDR).
  - Camera live preview for framing.
  - Battery voltage, CPU temperature, free disk space.
  - In-browser scan library (thumbnails, download PLY/E57).
- **Hardened button & status LED:** refactor `gpio_interrupt.py` into a debounced systemd service; multi-color LED / buzzer feedback (ready, scanning, error, shutdown).

### Success criteria
A user can set up the tripod, connect by phone, preview, trigger a scan, and download the cloud without touching a terminal.

---

## Phase 5 — Multi-Scan Registration & Advanced Reconstruction
**Objective:** Merge multiple station scans into an accurate, complete 3D scene.

- **Multi-station alignment** (`pilidar register`): global coarse (FPFH + RANSAC) then fine (point-to-plane ICP, color-ICP).
- **IMU gravity prior integration** — *leveling half ✅ done (v2.5.14)*; *registration constraint ⬜ open*:
  - ✅ Gravity vector / leveling plane from the MPU6050; every export is leveled to gravity. See [`docs/imu_leveling.md`](imu_leveling.md).
  - ⬜ Constrain registration to **yaw + horizontal translation** using the gravity direction (roll/pitch locked) — reduces drift and alignment failures.
- **Surface meshing & professional export:** Poisson reconstruction with normal estimation and density trimming; ASTM E57 with structured panoramic projections (CloudCompare, RealityCapture, Revit, Blender).

### Success criteria
Multiple overlapping scans align into a cohesive model with registration error under 15 mm in typical indoor scenes.

---

## Phase 6 — Packaging, Documentation & Community Release
**Objective:** Polish the project for external builders, makers, and contributors.

- **One-command setup:** `install.sh` (UART permissions, I2C, system deps like Hugin/libgl, Python packages).
- **Hardware & BOM refresh:** verify vendor links/pricing/alternatives; validate and release 3D-printable files (STEP/STL); wiring schematics for Pi 4 and Pi 5.
- **Documentation:** step-by-step build guide; software quickstart; troubleshooting & calibration FAQ.

### Success criteria
A community member can follow the build guide, assemble the hardware, run `install.sh`, and capture their first scan within 30 minutes.

---

## Priority Order
```
Phase 2 (open data + extrinsics)  →  Phase 3 (CLI pipeline)
        ↓                                   ↓
Phase 4 (web UI & field use)  ←─────────────┘
        ↓
Phase 5 (multi-scan, meshing, E57)
        ↓
Phase 6 (packaging, docs, release)
```

## Definition of Done (v1.0 Release)
- [ ] Automated test suite runs and passes in GitHub Actions CI. *(CI exists; keep it green)*
- [ ] Scan acquisition verified zero-drop at 921,600 baud on Raspberry Pi OS Bookworm. *(done on STL27L)*
- [ ] Scan data stored in open, versioned formats (no pickled objects).
- [ ] Calibration procedures for camera, LiDAR extrinsics, and stepper angles documented and repeatable.
- [ ] A single CLI command executes the full pipeline from raw capture to colorized point cloud.
- [ ] Scanner can be triggered wirelessly via the local Web UI over a mobile device.
- [ ] Multi-scan registration merges multiple rooms into a single cohesive scene.
- [ ] Point clouds export cleanly to industry-standard PLY and ASTM E57.
- [ ] Full BOM, 3D print models, and wiring diagrams published and validated.