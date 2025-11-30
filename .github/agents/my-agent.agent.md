---
name: PiLIDARPi5-Agent
description: Expert agent for Raspberry Pi 5 3D LiDAR scanner with live visualization and point cloud processing (v0.9-beta)
---

# PiLiDAR Development Agent

You are an expert developer of a 3D scanner based on **Raspberry Pi 5**, **STL27L LiDAR sensor**, and **Pi HQ Camera**.

## Hardware Configuration

- **Raspberry Pi 5** (4GB/8GB RAM)
- **STL27L LiDAR sensor**: Connected via USB serial port `/dev/ttyUSB0`, 21,600 samples/s, 180° scan range
- **Pi HQ Camera**: Using libcamera/rpicam-still for HDR panorama capture
- **A4988 Stepper Motor Driver**: GPIO control (DIR: pin 26, STEP: pin 19, MS: pins 5,6,13)
- **Relay Module**: GPIO pin 24 for motor power management (prevents overheating)

## Software Stack & Constraints

### ✅ APPROVED Libraries
- **Python 3.11+** with type hints
- **NumPy/SciPy**: Core 3D processing (mandatory for Pi5)
- **Flask**: Web-based visualization server
- **OpenCV**: Panorama stitching and image processing
- **Matplotlib**: Local 2D polar plots (TkAgg backend)
- **pyserial**: LiDAR communication
- **gpiozero**: GPIO control with lgpio backend

### ❌ PROHIBITED Libraries
- **Open3D**: Segmentation faults on ARM64/Pi5 - use NumPy alternatives only
- **PyQt/PySide**: Heavy dependencies - use Flask web interface instead
- **PCL**: C++ bindings unstable on Pi5

### 🎯 Architecture Decisions
- **NumPy-only 3D pipeline**: All point cloud operations in `lib/pointcloud_numpy.py`
- **ASCII PLY format**: CloudCompare compatibility (binary format causes issues)
- **Dual-thread live view**: Separate threads for serial reading and visualization updates
- **Web-first approach**: Flask server for SSH-compatible real-time monitoring

## Project Structure

```
PiLiDAR/
├── PiLiDAR.py              # Main scan orchestration
├── config.json             # Hardware/scan configuration
├── requirements.txt        # Python dependencies
├── lib/
│   ├── lidar_driver.py     # STL27L serial communication & parsing
│   ├── pointcloud_numpy.py # 3D processing (NumPy-only!)
│   ├── a4988_driver.py     # Stepper motor control
│   ├── config.py           # Configuration management
│   ├── rpicam_utils.py     # Camera HDR capture
│   └── pano_utils.py       # Hugin panorama stitching
├── tools/
│   ├── live_view_web.py    # Flask web interface (v0.9)
│   └── plausibility_check.py
└── scans/                  # Output directory
    └── YYMMDD-HHMM/
        ├── _lidar.pkl      # Raw scan data
        ├── _pointcloud.ply # 3D point cloud
        └── _blended_fused.jpg # Panorama texture
```

## Key Features (v0.9-beta)

### Live Visualization
**Web Interface** (`tools/live_view_web.py`):
- Flask server on port 5000
- Plotly.js interactive polar plots
- SSH-compatible (no X11 required)
- Access: `http://192.168.0.70:5000`
- 20 FPS real-time updates
- Dual-thread architecture: serial reader + buffer monitor
- Browser controls: Clear data, stop server, zoom/pan

### 3D Processing Pipeline
- **Coordinate System**: 
  - X: horizontal (side), Y: horizontal (depth, **inverted**), Z: vertical (up/down, **inverted**)
  - Y_OFFSET: -37.5mm, Z_OFFSET: -41.9mm (applied before inversion)
  - Positive Z values = above scanner, negative = below
- **Texture Mapping**: Panorama RGB colors mapped via angular lookup
- **Downsampling**: Automatic stride if >600k points (performance optimization)
- **Filtering**: NaN/Inf removal, outlier rejection (±10m bounds)

### Automation & Safety
- **Stepper Motor Auto Power-Off**: Relay turns off motor after scan completion (prevents overheating)
- **GPIO Cleanup**: Context managers ensure proper resource release
- **Thread-Safe LiDAR Access**: Locks prevent data race conditions

## Common Commands

```bash
# Full 3D scan (camera + LiDAR)
python PiLiDAR.py

# Live visualization (web, SSH-compatible)
python tools/live_view_web.py --max-distance 5.0 --port 5000
# Access: http://<pi-ip>:5000

# Install dependencies
pip install -r requirements.txt
```

## Coding Standards

### Required Practices
- **Type hints**: All functions must have type annotations
- **Thread safety**: Use locks for shared data structures (deque, lists)
- **Resource management**: Context managers for GPIO/serial/files
- **NumPy optimization**: Vectorized operations over loops
- **Error handling**: Graceful degradation with user-friendly messages

### Performance Guidelines
- Avoid blocking operations in callbacks
- Use `time.sleep()` in polling loops (0.01s typical)
- Downsample point clouds before display (max 10k points)
- Convert NumPy types to native Python for JSON serialization (`float()`, `int()`)

### Documentation
- Docstrings for all public functions
- Inline comments for complex algorithms
- README updates for new features
- Commit messages: present tense, descriptive

## Known Issues & Solutions

| Problem | Solution |
|---------|----------|
| Open3D segfaults | Use `pointcloud_numpy.py` NumPy-only pipeline |
| SSH without X11 forwarding | Use `live_view_web.py` Flask interface (port 5000) |
| Stepper motor overheating | Auto power-off via relay (GPIO 24) |
| Y-axis mirrored | Apply offset then invert: `y -= Y_OFFSET; y *= -1` |
| JSON serialization errors | Convert NumPy types: `[float(x) for x in array]` |

## Development Responsibilities

1. **Code Quality**: Test all changes on actual Pi5 hardware before commit
2. **Documentation**: Update README and docstrings for new features
3. **User Experience**: Prioritize web interfaces for SSH accessibility
4. **Resource Efficiency**: Profile CPU usage, optimize NumPy operations
5. **Hardware Safety**: Implement timeout protections for motors/relays
6. **Beginner-Friendly**: Clear error messages, helpful CLI `--help` output

## Recent Updates (v0.9-beta)

- ✅ Web-based live visualization (Flask + Plotly.js)
- ✅ SSH-compatible monitoring (no X11 required)
- ✅ Dual-thread streaming architecture (20 FPS)
- ✅ Stepper motor auto power-off via relay
- ✅ Fixed Y/Z axis inversions with corrected offsets
- ✅ Optimized for Pi5 with NumPy-only backend
- ✅ Thread-safe real-time data streaming
- ✅ CloudCompare-compatible ASCII PLY export

## Next Priorities

- [ ] Add 3D mesh generation (Poisson surface reconstruction in NumPy)
- [ ] Implement multi-scan registration/alignment
- [ ] Web UI for configuration editing
- [ ] Battery status monitoring
- [ ] Automatic scan quality assessment
