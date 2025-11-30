# PiLiDAR Development Roadmap
**Version 0.9-beta Baseline → Future Versions**

## Current Capabilities (v0.9-beta)

### ✅ Working Features
- **Real-time Visualization**
  - Flask web server (SSH-compatible, port 5000)
  - Matplotlib 2D view (TkAgg, requires X11)
  - Dual-thread streaming architecture (20 FPS)
  - Thread-safe buffer management with locks

- **3D Point Cloud Processing (NumPy-only)**
  - 180° scan assembly with stepper motor rotation
  - Y/Z axis inversions with offset corrections
  - ASCII PLY export for CloudCompare
  - NaN/Inf filtering, ±10m outlier bounds

- **Hardware Control**
  - STL27L/LD06/LD19 LiDAR support (921600 baud)
  - A4988 stepper motor driver (GPIO 23/24/25)
  - Relay-based motor auto power-off
  - IMU orientation tracking (MPU6050)

- **Image Processing**
  - 360° panorama stitching (4/8/16 exposures)
  - Hugin integration for HDR blending
  - Raspberry Pi Camera Module 3 support

### ❌ Open3D-Dependent Features (Pi5 Incompatible)
- Global registration (RANSAC + FPFH features)
- ICP refinement (point-to-plane, colored ICP)
- Surface meshing (Poisson, alpha shapes, ball pivoting)
- Statistical outlier removal (k-nearest neighbors)
- Normal estimation and downsampling
- Laplacian smoothing and mesh optimization

---

## Version 1.0 - NumPy Core Implementation (Q1 2026)

### Priority Matrix
| Feature | Impact | Effort | Priority |
|---------|--------|--------|----------|
| Web UI Config Editor | High | Low | **P0** |
| NumPy Outlier Removal | High | Low | **P0** |
| Scan Quality Assessment | High | Low | **P0** |
| Battery Monitoring | Medium | Low | P1 |
| RANSAC Registration | High | High | P1 |
| Multi-scan Merge | High | Medium | P1 |

### P0 Features (v1.0.0)
**Web-Based Configuration Editor**
- Browser interface for `config.json` editing
- Live validation with parameter constraints
- Offset calibration wizard with visual feedback
- No SSH/nano required for beginners
- Implementation: Flask routes + HTML/JavaScript frontend

**NumPy Statistical Outlier Removal**
- Replace `open3d.geometry.PointCloud.remove_statistical_outlier()`
- scipy.spatial.KDTree for k-nearest neighbors
- Standard deviation threshold filtering
- 10-100x faster than Open3D on ARM64
- Integration: `lib/pointcloud_numpy.py`

**Scan Quality Assessment**
- Real-time metrics: point density, coverage percentage, noise level
- Web dashboard display during acquisition
- Automatic rescan triggers for low-quality data
- Integration: `tools/live_view_web.py`

### P1 Features (v1.2-1.3)
**RANSAC Registration (NumPy)**
- Fast Point Feature Histograms (FPFH) in pure NumPy
- Random sample consensus for coarse alignment
- 33-dim feature descriptors per point
- Voxel downsampling for performance
- Target: <30s for 100k point clouds

**Multi-Scan Automatic Merge**
- Sequential alignment with overlap detection
- Transformation matrix accumulation
- Conflict resolution for overlapping regions
- Progress tracking in web UI

**Alpha Shape Meshing (NumPy)**
- Delaunay triangulation via scipy.spatial
- Alpha parameter optimization (adaptive)
- Triangle normal consistency checks
- PLY/OBJ mesh export

---

## Version 1.5 - Advanced Reconstruction (Q3 2026)

### P1 Features
**ICP Refinement (NumPy)**
- Point-to-plane iterative closest point
- KDTree-accelerated correspondence search
- Robust M-estimator for outliers (Huber, Tukey)
- Multi-scale pyramid registration
- Target: <5mm alignment error

**Poisson Surface Reconstruction (NumPy)**
- Screened Poisson solver (scipy.sparse.linalg)
- Octree spatial subdivision
- Normal field integration
- Depth parameter: 8-12 (adaptive)

**Normal Estimation (NumPy)**
- PCA-based surface normal calculation
- Oriented normal propagation
- k-nearest neighbors: 20-50 points
- Integration: `lib/pointcloud_numpy.py`

### P2 Features
**Voxel Downsampling**
- Grid-based spatial hashing
- Centroid/random point selection modes
- Configurable voxel size (1-50mm)

**Radius Outlier Removal**
- Sphere-based neighbor counting
- scipy.spatial.cKDTree for speed
- Adjustable radius and min_neighbors

---

## Version 2.0 - Web Platform & Collaboration (Q1 2027)

### P0 Features
**3D Web Viewer**
- Three.js-based point cloud renderer
- Client-side PLY parsing (Web Workers)
- Interactive camera controls (orbit, pan, zoom)
- Point size/color/opacity sliders
- Measurement tools (distance, area)

**Session Management**
- Database backend (SQLite → PostgreSQL)
- Scan history with thumbnails
- Tags, notes, project organization
- Export bundles (PLY + config + images)

**User Authentication**
- Multi-user support with roles
- OAuth2 integration (GitHub, Google)
- Per-scan access permissions
- API tokens for automation

### P1 Features
**Collaborative Editing**
- Real-time point cloud annotations
- Measurement sharing between users
- Comment threads on 3D features
- WebSocket-based synchronization

**REST API**
- Scan upload/download endpoints
- Registration job queue (Celery)
- Webhook notifications
- OpenAPI/Swagger documentation

---

## Version 2.5 - Mobile & Edge (Q3 2027)

### Features
**Progressive Web App (PWA)**
- Offline-capable web interface
- Push notifications for scan completion
- Mobile-responsive UI (tablet/phone)
- Camera access for on-site photos

**Edge Processing Optimization**
- ARM NEON SIMD vectorization
- Multi-core NumPy parallelization (joblib)
- Adaptive quality modes (fast/balanced/quality)
- Thermal throttling detection

**Bluetooth Remote Control**
- BLE GATT server on Pi5
- Android/iOS companion app
- One-button scan trigger
- Battery level monitoring

**Battery Management**
- UPS HAT integration (Geekworm X1000)
- Voltage/current monitoring (INA219)
- Auto-shutdown at 10% capacity
- Web UI battery indicator

---

## Version 3.0 - AI & Automation (2028+)

### P2-P3 Features (Exploratory)
**ML-Based Registration**
- Feature learning via small neural networks
- ONNX Runtime for ARM64 inference
- Training pipeline for custom datasets
- Fallback to classical RANSAC

**Semantic Segmentation**
- PointNet++ lite model
- Object classification (ground/vegetation/building)
- Color-coded visualization
- Export labeled point clouds

**SLAM Integration**
- Visual-LiDAR odometry
- Loop closure detection
- Incremental map building
- ROS2 compatibility layer

**GPU Acceleration**
- Vulkan compute shaders (RPi5 GPU)
- OpenCL kernels for registration
- CUDA fallback for edge devices (Jetson Nano)

---

## Technical Debt & Maintenance

### Continuous Tasks
- **Code Quality**
  - Type hints for all public APIs
  - Docstrings (NumPy format)
  - Unit tests (pytest, >80% coverage)
  - Integration tests for hardware

- **Performance Monitoring**
  - Profiling with cProfile
  - Memory leak detection (tracemalloc)
  - Benchmark suite (pytest-benchmark)

- **Documentation**
  - API reference (Sphinx)
  - Hardware assembly guide
  - Calibration procedures
  - Troubleshooting flowcharts

- **Dependency Management**
  - Monthly security audits (pip-audit)
  - NumPy/SciPy version pinning
  - ARM64 wheel availability checks

---

## Migration Strategy: Open3D → NumPy

### Phase 1: Feature Parity (v1.0-1.5)
1. **Geometry Processing**
   - Outlier removal → scipy.spatial.KDTree
   - Downsampling → voxel grid hashing
   - Normal estimation → PCA covariance

2. **Registration**
   - RANSAC → NumPy random sampling + SVD
   - ICP → KDTree correspondence + Kabsch algorithm
   - Colored ICP → Weighted SVD with RGB terms

3. **Meshing**
   - Alpha shapes → Delaunay + edge length filter
   - Poisson → scipy.sparse linear solver
   - Ball pivoting → Incremental sphere rolling

### Phase 2: Optimization (v2.0+)
- Vectorization: NumPy broadcasting patterns
- Parallelization: joblib for multi-core CPU
- Caching: LRU cache for expensive operations
- Memory: In-place operations, view slicing

### Phase 3: Validation (All versions)
- Unit tests: Compare outputs with Open3D (on x86_64)
- Accuracy metrics: Hausdorff distance, RMSE
- Performance benchmarks: Wall time, peak memory
- Visual inspection: Side-by-side PLY comparisons

---

## Success Metrics

### v1.0 Goals
- ✅ 100% Open3D dependency removed
- ✅ Web UI usable without SSH
- ✅ Scan completion <5 minutes (180° × 200 steps)
- ✅ Registration accuracy <5mm (indoor scenes)

### v2.0 Goals
- ✅ Multi-user support (5+ concurrent)
- ✅ 3D viewer loads <10s (500k points)
- ✅ REST API response time <200ms
- ✅ Mobile UI responsive (tablet/phone)

### v3.0 Goals
- ✅ SLAM real-time (10 Hz pose updates)
- ✅ ML inference <500ms/scan
- ✅ GPU acceleration 5x speedup
- ✅ ROS2 compatibility

---

## Community & Ecosystem

### Open Source Strategy
- **GitHub Releases**
  - Semantic versioning (major.minor.patch)
  - Changelog with breaking changes
  - Pre-built ARM64 wheels (GitHub Releases)

- **Documentation**
  - Read the Docs hosting
  - Video tutorials (YouTube)
  - Example projects gallery

- **Support Channels**
  - GitHub Discussions for Q&A
  - Discord server for real-time help
  - Issue templates for bug reports

### Hardware Partnerships
- Compatible LiDAR list (tested models)
- 3D-printable enclosure designs (STL files)
- Recommended BOM with supplier links

---

## Risk Assessment

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| NumPy ICP slower than Open3D | High | Medium | SIMD optimization, Cython |
| Pi5 thermal throttling | Medium | High | Heatsink required, adaptive quality |
| Battery runtime <30min | Medium | Medium | Larger capacity, power profiling |
| Web UI security vulnerabilities | High | Low | HTTPS, CSRF tokens, input validation |
| Breaking API changes in SciPy | Low | Low | Version pinning, deprecation warnings |

---

## Next Steps

### Immediate Actions (This Week)
1. **User Decision Required**: Select v1.0 priority feature
   - Option A: Web UI Config Editor (best UX improvement)
   - Option B: NumPy Outlier Removal (fastest performance win)
   - Option C: Scan Quality Assessment (data reliability)

2. **Development Setup**
   - Create feature branch: `feature/v1.0-{selected-feature}`
   - Update roadmap document with milestone dates
   - Set up issue tracking (GitHub Projects)

3. **Prototyping**
   - Spike: Benchmark scipy.spatial.KDTree vs Open3D
   - Proof of concept: Flask config editor mockup
   - Test: Battery monitoring GPIO ADC reading

### Monthly Milestones (Q1 2026)
- **January**: P0 features implementation
- **February**: P1 features + user testing
- **March**: v1.0.0 release candidate + documentation

---

## Questions for User

1. **Priority Selection**: Which P0 feature should be implemented first?
   - Web UI Config Editor (ease of use)
   - NumPy Outlier Removal (performance)
   - Scan Quality Assessment (reliability)

2. **Hardware Roadmap**: Plan to add battery/UPS HAT?
   - If yes, which model? (Geekworm X1000, Waveshare UPS HAT)

3. **Collaboration Features**: Interest in multi-user support (v2.0)?
   - Personal use only → Skip authentication
   - Lab/team use → Add user management

4. **Performance Targets**: Acceptable scan time?
   - Current: ~5 minutes (200 steps × 180°)
   - Target: <3 minutes? (requires faster motor)

5. **API Requirements**: Need programmatic access?
   - If yes, prioritize REST API in v2.0
   - If no, focus on web UI features
