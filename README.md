# PiLiDAR - DIY 360° 3D Panorama Scanner
## _WORK IN PROGRESS_

PiLiDAR is an open-source, low-cost DIY 360° LiDAR scanner and panoramic 3D
reconstruction system. It captures 2D LiDAR planes while rotating on a
stepper-driven turntable, shoots a 360° fisheye panorama, and assembles a
vertex-colorized 3D point cloud.

## Core Features:
- **LiDAR**: custom serial driver for LDRobot **LD06**, **LD19** or **STL27L**
    - CRC package integrity check
    - [Hardware PWM](https://github.com/Pioreactor/rpi_hardware_pwm) calibrated using curve fitting
    - 2D live visualization and export (numpy or CSV)

- **Panorama**: 6K 360° spherical map
    - stitched from fisheye photos using [**Hugin** Panorama photo stitcher](https://hugin.sourceforge.io/)
    - constant camera exposure by reading EXIF data of automatic
    - constant white balance by iterative optimization of color gains

- **3D Scene**: assembly of 3D scenes from 2D planes based on angle and offsets
    - sampling **vertex colors from panorama**
    - Open3D visualization and export (PCD, PLY or [e57](https://github.com/davidcaron/pye57))
    - aligning multiple scenes using **global registration** and **ICP fine-tuning**
    - **Poisson Surface Meshing** (very slow on Pi4, recommended to run on PC)

## Coordinate space

Everything is assembled in **object space** — the scanner's own frame, fixed to
the turntable and **independent of gravity**. Gravity leveling is a separate,
later step (see [docs/imu_leveling.md](docs/imu_leveling.md)).

| axis | meaning |
|---|---|
| **+Z** | scanner **up** = turntable rotation axis (floor at −Z, ceiling at +Z) |
| **+Y** | **forward** — the direction the camera looks at stepper angle 0° |
| **X** | the remaining horizontal axis (left / right) |

- The **camera** nodal point sits on the rotation axis at the origin and looks
  **+Y** (pan = 0° by definition; its tilt/roll come from the stitched panorama).
- The **LiDAR** faces **−Y** (mounted back-to-back with the camera). At stepper 0°
  its vertical disc lies in the **X–Z plane**; the disc **roll** axis is **Y**.
  As the turntable turns, the disc revolves about +Z and the point cloud is built
  by revolving each fan about `(0,0,1)`.
- **LiDAR mount** (`3D.CALIBRATION` in `config.json`): `POSITION` = [x, y, z] mm in
  scanner space (Y = forward/mount axis, Z = height above the camera); `ROTATION`
  = [rx, ry, rz] degrees — the **roll about Y** (`ry`) rotates the upright disc in
  its own plane. The offset within the disc plane (X) and the other rotations are
  assumed 0.
- The camera's **panorama** is the angular reference: cloud → equirect uses
  `VERTEXCOLOUR.CENTER_M` + `ROTATE_DEG` (see `lib.pointcloud.projection_params`),
  the same projection used for the LiDAR intensity panorama and for vertex colors.

## preliminary results
single scans, no registration, no post processing.
klick the images to open the pointclouds in Sketchfab.

[![Exterior](images/exterior.jpeg)](https://sketchfab.com/models/7997b63a3cb747f99b8f161862318bec/embed?autostart=1&ui_animations=0&ui_stop=0&ui_inspector=0&ui_watermark_link=0&ui_watermark=0&ui_ar=0&ui_help=0&ui_settings=0&ui_vr=0&ui_fullscreen=0&ui_annotations=0)

*Exterior Scan (colormapped Intensity)*

[![Interior](images/interior.jpeg)](https://sketchfab.com/models/0311c098c57b458abe3a5d3dda9fe92b/embed?autospin=0&autostart=1&ui_animations=0&ui_inspector=0&ui_watermark_link=0&ui_watermark=0&ui_ar=0&ui_help=0&ui_settings=0&ui_vr=0&ui_fullscreen=0&ui_annotations=0)

*Interior Scan (Vertex Colors)*


## Hardware Specs / Bill of Materials:

- Lidar:
  - One of:
    - LDRobot LD06: $80 ([link](https://www.aliexpress.us/item/3256803352905216.html))
    - LDRobot LD19: $70 ([link](https://www.amazon.com/DTOF-D300-Distance-Obstacle-Education/dp/B0B1V8D36H))
    - LDRobot STL27L: $160 ([link](https://www.dfrobot.com/product-2726.html))
- Camera and Lens:
  - Raspberry Pi HQ Camera with ArduCam M12 Lens: $60 ([M25156H18, p.7](https://www.arducam.com/doc/Arducam_M12_Lens_Kit_for_Pi_HQ_Camera.pdf)) ([link](https://www.arducam.com/doc/Arducam_M12_Lens_Kit_for_Pi_HQ_Camera.pdf))
- Raspberry Pi 4: $50
- NEMA17 42-23 stepper with A4988 driver: $10 ([link](https://www.amazon.com/SIMAX3D-Nema17-Stepper-Motor/dp/B0CQLFNSMJ))
- Power Supply:
  - One of:
    - [v1]: 2x _18650_ Batteries (7.2V) with step-down converter
    - [v2]: 10.000 mAh USB Powerbank with step-up converter

*Total:* \~ $200 - $280 (in April of 2025) (not including power supply)

(links are only for example, not necessarily the recommended way to purchase)

![PiLiDAR v1](images/pilidar_covershot.jpg)
*Rev. 1 using 2x 18650 Batteries and Buck Converter*


![PiLiDAR v2](images/pilidar_covershot_v2.jpg)
*Rev. 2 using 10.000 mAh Powerbank and Boost Converter*


## Quickstart

1. **Set up the Python environment** (venv, desktop and Pi): see [docs/setup.md](docs/setup.md#python-environment-venv)
2. **Wire the hardware** and configure GPIO/I2C/UART: see [docs/hardware.md](docs/hardware.md#wiring) and [docs/setup.md](docs/setup.md)
3. **Run a scan**: `~/venv/bin/python PiLiDAR.py` (or press the scan button)

## CLI

A unified `pilidar` CLI (`lib/cli.py`) wraps the common tasks (Phase 3 in progress):

```bash
pilidar doctor                      # pre-flight hardware diagnostics
pilidar process <scan_id>           # assemble + colour + level a raw scan -> .ply
pilidar convert [scan_id ...]       # convert legacy .pkl scans to the open NPZ+JSON container
pilidar export cloud.ply cloud.pcd  # convert a point cloud between formats
```

Without installing (`pip install -e .`), run it as `python pilidar.py <command>`.

## Downloading a scan from the Pi

Scans are written under `~/PiLiDAR/scans/<scan_id>/` on the Pi. Pull one (or all)
down to your local `scans/` with `rsync`:

```bash
# one scan
rsync -az --info=progress2 pi@192.168.1.16:/home/pi/PiLiDAR/scans/<scan_id>/ scans/<scan_id>/

# everything, and delete local files that no longer exist on the Pi
rsync -az --delete --info=progress2 pi@192.168.1.16:/home/pi/PiLiDAR/scans/ scans/
```

(`-a` preserves times/permissions, `-z` compresses, `--info=progress2` shows
overall progress. The scan files are world-readable, so no `sudo` is needed.)

## Calibrate extrinsics

```python
experiments/calibrate_extrinsics.py 261014-1954
```

## Documentation

| Topic | File |
|---|---|
| Setup: Python env, GPIO, UART, PWM, Hugin, MeshLab, USB power | [docs/setup.md](docs/setup.md) |
| Bill of materials, wiring, LiDAR specs, 3D printing | [docs/hardware.md](docs/hardware.md) |
| Serial protocol, visualization, USB scan dumping | [docs/protocol.md](docs/protocol.md) |
| IMU gravity leveling (frames, method, accuracy) | [docs/imu_leveling.md](docs/imu_leveling.md) |
| LiDAR geometry (the open left/right fan-half offset) | [docs/lidar_geometry.md](docs/lidar_geometry.md) |
| Troubleshooting & references | [docs/troubleshooting.md](docs/troubleshooting.md) |
| Project roadmap (phases 0–6) | [docs/roadmap.md](docs/roadmap.md) |
| Change history | [CHANGELOG.md](CHANGELOG.md) |

## Repos:
Code: https://github.com/PiLiDAR/PiLiDAR
Hardware: https://github.com/PiLiDAR/PiLiDAR-Hardware
