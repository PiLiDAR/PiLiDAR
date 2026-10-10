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
ssh-copy-id -i ~/.ssh/id_ed25519.pub flip@192.168.1.75
```


## Overview
PiLiDAR is an open-source, low-cost DIY 360° LiDAR scanner and panoramic 3D reconstruction system. This roadmap guides the project from an experimental research prototype into a robust, repeatable, and maintainable platform ready for community use and field deployment.

The project follows a disciplined engineering pipeline:
**Capture -> Validate -> Calibrate -> Process -> Register -> Export**

### Phases Summary
- **[Phase 0 — Foundation, CI/CD, and Mock Testing Harness](#phase-0--foundation-cicd-and-mock-testing-harness):** Modern tooling (`pyproject.toml`), decouple from physical hardware with mock harnesses, and establish automated regression testing.
- **[Phase 1 — Hardware Abstraction & High-Speed Acquisition Stack](#phase-1--hardware-abstraction--high-speed-acquisition-stack):** Zero-drop multithreaded serial reader at 921,600 baud, modern GPIO (`gpiozero`/`lgpio`), native `Picamera2`, and pre-flight hardware diagnostics.
- **[Phase 2 — Open Data Standards & Geometric Calibration](#phase-2--open-data-standards--geometric-calibration):** Replace pickle storage with open containers (NPZ/HDF5/PLY/LAS) and formalize LiDAR-to-camera 6-DOF extrinsics calibration.
- **[Phase 3 — Unified CLI & Processing Pipeline Engine](#phase-3--unified-cli--processing-pipeline-engine):** Consolidate ad-hoc scripts into a unified `pilidar` CLI and deterministic end-to-end processing pipeline.
- **[Phase 4 — Field Usability: Local Web UI & Remote Operation](#phase-4--field-usability-local-web-ui--remote-operation):** Lightweight mobile web controller over Wi-Fi hotspot and hardened hardware status indicators for untethered tripod scanning.
- **[Phase 5 — Multi-Scan Registration & Advanced Reconstruction](#phase-5--multi-scan-registration--advanced-reconstruction):** Multi-station point cloud alignment, IMU gravity vector leveling, Poisson meshing, and ASTM E57 export.
- **[Phase 6 — Packaging, Documentation & Community Release](#phase-6--packaging-documentation--community-release):** Automated `install.sh` bootstrap, updated BOM, verified 3D CAD/print files, and comprehensive maker build guide.

---

## Phase 0 — Foundation, CI/CD, and Mock Testing Harness
**Timeline:** 2–3 weeks  
**Objective:** Modernize tooling, decouple development from physical hardware, and establish automated regression testing.

### Goals
- Allow developers and contributors to build, test, and verify algorithms without physical scanner hardware attached.
- Establish clean packaging, linting, and continuous integration.
- Prevent regression while refactoring legacy components.

### Tasks
- **Modern Python Packaging**
  - Transition from loose scripts and unpinned `requirements.txt` to standard `pyproject.toml`.
  - Define explicit dependency groups: `pi-hardware` (GPIO, camera), `processing` (Open3D, OpenCV, SciPy), and `dev` (pytest, ruff).
  - Modernize platform detection in `lib/platform_utils.py` using kernel and device-tree checks instead of brittle architecture strings.
- **Hardware Mocking & Packet Replay Harness**
  - Build `MockLidarSerial` to simulate serial streams by replaying recorded raw byte streams for LD06 and STL27L.
  - Implement a mock camera and mock stepper driver for desktop simulation and headless environments.
  - Check in a minimal, anonymized synthetic sample scan dataset under `tests/fixtures/`.
- **Automated Testing & CI Pipeline**
  - Implement `pytest` test suite covering:
    - CRC8 validation and corrupted packet recovery.
    - Polar-to-Cartesian transforms and geometric offsets.
    - Point cloud loading, downsampling, and export formatting.
  - Add GitHub Actions CI workflow for automated linting (`ruff`), type checks, and running test suites across Linux, macOS, and Windows.

### Deliverables
- `pyproject.toml` configuration and clean virtual environment setup.
- Mock serial/stepper/camera test fixtures.
- Working `pytest` suite running in automated GitHub Actions CI.

### Success criteria
The test suite passes in CI on desktop machines without physical hardware or Raspberry Pi peripherals connected.

---

## Phase 1 — Hardware Abstraction & High-Speed Acquisition Stack
**Timeline:** 4–6 weeks  
**Objective:** Harden the acquisition layer for modern Raspberry Pi OS, eliminate serial packet drops at high baudrates, and ensure reliable motion control.

### Goals
- Support Raspberry Pi OS Bookworm (Debian 12) and Raspberry Pi 4/5 hardware.
- Ensure 100% packet integrity at 921,600 baud without buffer overruns.
- Provide deterministic stepper motion and responsive hardware controls.

### Tasks
- **High-Speed Serial Driver Overhaul**
  - Replace byte-by-byte reads (`read()`) with buffered block reads and sliding window frame parsers.
  - Implement a dedicated background reader thread with a bounded thread-safe queue to decouple serial I/O from processing.
  - Add automated sync-byte recovery, timeout handling, and malformed packet error logging.
  - Benchmark and validate sustained throughput across all supported models: LD06, LD19, and STL27L.
- **Modern GPIO & Motion Control**
  - Migrate from deprecated `RPi.GPIO` to modern `gpiozero` / `lgpio` (compatible with Pi 4, Pi 5, and kernel 6.x+).
  - Investigate hardware timer / PWM stepping to eliminate userspace `time.sleep` jitter during sweeps.
  - Evaluate endstop or home-position switch support to prevent cumulative stepper angle drift.
- **Camera Subsystem Modernization**
  - Migrate from repeated `rpicam-still` subprocess invocations to native `Picamera2` Python bindings.
  - Maintain persistent sensor state to eliminate repeated initialization delays between exposure brackets.
- **Pre-Flight Hardware Diagnostics**
  - Implement an automated diagnostic routine (`pilidar doctor`) checking UART communication, I2C IMU connectivity, camera availability, and storage space before beginning a scan.

### Deliverables
- Zero-drop, multithreaded serial driver for LD06/STL27L.
- Modernized GPIO and stepper driver module.
- Hardware diagnostic utility (`pilidar doctor`).

### Success criteria
A full 180° sweep with the STL27L captures continuously at 921,600 baud with 0 dropped packets and consistent angular step intervals.

---

## Phase 2 — Open Data Standards & Geometric Calibration
**Timeline:** 3–4 weeks  
**Objective:** Replace unversioned pickle storage with open, standardized file formats and establish rigorous calibration procedures.

### Goals
- Eliminate insecure and Python-locked `.pkl` scan files.
- Enable full transparency and cross-platform inspection of raw and processed scans.
- Formally measure and model the physical geometric relationships between LiDAR, camera, and the rotation axis.

### Tasks
- **Standardized Scan Container & Metadata**
  - Deprecate `pickle.dump` / `pickle.load` for scan persistence.
  - Adopt open raw containers: compressed NPZ / HDF5 for raw time-series range and angle packets.
  - Adopt standardized 3D outputs: PLY and LAS formats with intensity attributes and structured `metadata.json` sidecars (recording calibration parameters, timestamps, hardware revisions, and software versions).
- **LiDAR-to-Camera 6-DOF Extrinsics Calibration**
  - Formalize rigid-body transform matrix ($X, Y, Z, \text{roll}, \text{pitch}, \text{yaw}$) between LiDAR optical center, stepper rotation axis, and camera sensor center.
  - Build a semi-automated calibration utility using a known physical fixture (e.g. corner target or planar board).
- **Camera Intrinsic & Panorama Exposure Normalization**
  - Document and automate lens distortion calibration (fisheye / equirectangular projection parameters).
  - Standardize auto-exposure and white balance normalization routines to eliminate seam artifacts in 360° panoramas.

### Deliverables
- Versioned scan directory schema with `metadata.json` specification.
- Geometric calibration scripts and step-by-step calibration guide.
- Backward compatibility converter for legacy `.pkl` scans.

### Success criteria
Scans can be opened and visualized in third-party tools (CloudCompare, MeshLab, Blender) without requiring Python or PiLiDAR source code.

---

## Phase 3 — Unified CLI & Processing Pipeline Engine
**Timeline:** 3–4 weeks  
**Objective:** Replace scattered root scripts with a clean, modular CLI and a deterministic end-to-end processing pipeline.

### Goals
- Replace ad-hoc scripts (`filter_3D.py`, `process_3D.py`, `meshing_test.py`) with a single command-line interface.
- Provide deterministic, single-command processing from raw scan to colorized point cloud.

### Tasks
- **Unified CLI Application**
  - Consolidate tools under a modular CLI (e.g. `pilidar` using `typer` or `argparse`):
    - `pilidar capture`: Run acquisition workflow with configurable flags.
    - `pilidar process`: Assemble point clouds, project panorama vertex colors, and apply filters.
    - `pilidar export`: Convert and export scans to PLY, LAS, PCD, or E57.
    - `pilidar calibrate`: Run hardware calibration workflows.
- **Point Cloud Preprocessing & Noise Rejection**
  - Implement statistical outlier removal (SOR) and radius outlier removal (ROR).
  - Add intensity threshold filtering to discard weak returns and ambient optical reflections.
  - Implement voxel grid downsampling with preserved color and intensity attributes.
- **Pipeline Automation & Progress Reporting**
  - Implement clear terminal progress bars and structured logging.
  - Support batch processing across multiple raw scan directories.

### Deliverables
- Installable `pilidar` CLI package.
- Clean end-to-end processing pipeline module.
- Documented CLI reference guide.

### Success criteria
Running `pilidar process <scan_dir>` turns a raw capture folder into a clean, vertex-colorized 3D point cloud with a single command.

---

## Phase 4 — Field Usability: Local Web UI & Remote Operation
**Timeline:** 3–4 weeks  
**Objective:** Make PiLiDAR practical in the field on a tripod without requiring an SSH session or tethered monitor.

### Goals
- Enable untethered field operation via smartphone or tablet.
- Provide visual feedback and system health monitoring during scanning.

### Tasks
- **Lightweight Local Web Controller**
  - Build a lightweight backend (FastAPI / aiohttp) designed to run on the Raspberry Pi.
  - Implement a mobile-responsive web dashboard accessible over the Pi's Wi-Fi hotspot (`http://pilidar.local`):
    - Real-time scanner state (Idle, Scanning, Stitching, Complete, Error).
    - Trigger scans and select presets (Quick Preview, High Resolution, HDR).
    - Camera live preview for framing before initiating a sweep.
    - Battery voltage, CPU temperature, and available disk space indicators.
    - In-browser scan library to inspect thumbnails and download exported PLY/E57 files.
- **Hardened Hardware Button & Status LED**
  - Refactor `gpio_interrupt.py` into a standard systemd service with debounced inputs.
  - Add support for multi-color status LEDs or buzzer feedback (ready, scanning, error, shutdown).

### Deliverables
- Headless Web UI application and systemd service files.
- Refactored hardware button and LED daemon.
- Hotspot configuration documentation.

### Success criteria
A user can set up the scanner on a tripod, connect via mobile phone, preview framing, trigger a scan, and download the finished point cloud without ever opening a terminal.

---

## Phase 5 — Multi-Scan Registration & Advanced Reconstruction
**Timeline:** 4–5 weeks  
**Objective:** Merge multiple station scans into an accurate, complete 3D scene.

### Goals
- Combine point clouds from different tripod locations into a single coordinate system.
- Support professional export formats for BIM, CAD, and VFX workflows.

### Tasks
- **Multi-Station Alignment Pipeline**
  - Generalize registration workflows into a multi-scan alignment tool (`pilidar register`).
  - Implement two-stage registration:
    - Global coarse alignment: FPFH feature extraction and RANSAC matching.
    - Fine alignment: Point-to-Plane ICP and Color-ICP refinement.
- **IMU Gravity Prior Integration**
  - Utilize MPU6050 accelerometer data to calculate the scanner's gravity vector (leveling plane).
  - Constrain registration transformations to yaw and horizontal translation, dramatically reducing drift and alignment failure.
- **Surface Meshing & Professional Export**
  - Implement Poisson surface reconstruction with normal estimation and density trimming.
  - Standardize ASTM E57 export including structured panoramic image projections compatible with CloudCompare, RealityCapture, Revit, and Blender.

### Deliverables
- Multi-scan registration command and alignment error reporting.
- E57 export module with embedded spherical panoramic imagery.
- Poisson mesh generation pipeline.

### Success criteria
Multiple overlapping scans align into a cohesive 3D model with measurable registration error under 15 mm in typical indoor test scenes.

---

## Phase 6 — Packaging, Documentation & Community Release
**Timeline:** 2–3 weeks  
**Objective:** Polish the project for external builders, makers, and contributors.

### Goals
- Enable any maker with a 3D printer and soldering iron to build a working PiLiDAR scanner.
- Ensure smooth installation on fresh Raspberry Pi OS installations.

### Tasks
- **One-Command Setup & Bootstrap Script**
  - Create an automated setup script (`install.sh`) to configure UART permissions, I2C, system dependencies (Hugin, libgl), and python packages.
- **Hardware & Bill of Materials Refresh**
  - Verify and update vendor links, current pricing, and component alternatives in `README.md`.
  - Validate and release 3D printable files (STEP/STL) for the main body and planetary gearbox.
  - Create clear wiring schematics for Raspberry Pi 4 and Raspberry Pi 5.
- **Comprehensive Documentation**
  - Step-by-step Build Guide (assembly, wiring, flashing OS).
  - Software Quickstart Guide.
  - Troubleshooting & Calibration FAQ.

### Deliverables
- One-line automated install script.
- Complete hardware CAD repository and build guide.
- Tagged `v1.0.0` release candidate.

### Success criteria
A community member can follow the build guide, assemble the hardware, run `install.sh`, and capture their first scan within 30 minutes of hardware assembly.

---

## Priority Order & Execution Strategy

```
Phase 0 (Foundation & CI)
    ↓
Phase 1 (Hardware & UART) ──→ Phase 2 (Data & Calibration)
    ↓                              ↓
Phase 4 (Web UI & Field Usability) ← Phase 3 (Unified CLI Pipeline)
                                           ↓
                                     Phase 5 (Multi-Scan & E57)
                                           ↓
                                     Phase 6 (Docs & Release)
```

1. **Phase 0 comes first:** Never refactor hardware drivers without automated tests and mock fixtures in place, otherwise desktop contributors cannot assist.
2. **Phase 1 & Phase 2 establish the baseline:** Acquisition reliability and geometry correctness are the core physics of the system.
3. **Phase 3 consolidates the software:** Eliminates script fragmentation and locks the public CLI contract.
4. **Phase 4 enables real-world testing:** Makes field testing fast, pleasant, and independent of desktop computers.
5. **Phase 5 & 6 provide professional Polish:** Delivers multi-room scanning, standard BIM/E57 exports, and complete community documentation.

---

## Definition of Done (v1.0 Release)
The project will be considered ready for a milestone `v1.0` release when:
- [ ] Automated test suite runs and passes in GitHub Actions CI.
- [ ] Scan acquisition is verified zero-drop at 921,600 baud on Raspberry Pi OS Bookworm.
- [ ] Scan data is stored in open, versioned formats (no pickled objects).
- [ ] Calibration procedures for camera, LiDAR extrinsics, and stepper angles are documented and repeatable.
- [ ] A single CLI command executes the full pipeline from raw capture to colorized point cloud.
- [ ] The scanner can be triggered wirelessly via the local Web UI over a mobile device.
- [ ] Multi-scan registration successfully merges multiple rooms into a single cohesive scene.
- [ ] Point clouds export cleanly to industry-standard PLY and ASTM E57 formats.
- [ ] Full bill of materials, 3D print models, and wiring diagrams are published and validated.
