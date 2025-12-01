# Scan Automation Implementation

## Overview
Implemented "Start Full Scan" button in web dashboard to automate the complete 3D scanning workflow.

## Features

### 1. Web Interface Button
- **Location**: Main dashboard at `http://192.168.0.70:5000`
- **Icon**: 🚀 Start Full Scan
- **Description**: Execute complete 3D scan workflow: Camera panorama capture, LiDAR scanning, stitching, and point cloud generation
- **Status Display**: Real-time status updates during scan execution

### 2. Backend Implementation

#### API Endpoints

**POST /api/scan/start**
- Starts full scan by executing `PiLiDAR.py` as subprocess
- Validates no scan is already running (409 conflict if running)
- Generates unique scan ID based on timestamp (YYMMDD-HHMM)
- Returns scan ID and success status

**GET /api/scan/status?scan_id={id}**
- Polls current scan status
- Returns:
  - `status`: idle, running, completed, failed
  - `progress`: Current step description
  - `error`: Error message if failed
  - `points`: Point count when available
  - `output_dir`: Scan output directory when completed

#### Progress Monitoring
The system monitors scan output and updates progress based on keywords:
- "Camera capture" → "Camera panorama capture..."
- "LiDAR scan" → "LiDAR 180° scan..."
- "Stitching" → "Stitching panorama..."
- "Processing 3D" → "3D point cloud processing..."
- "Saving" → "Saving results..."
- Auto-extracts point count from output

### 3. User Experience

#### Workflow
1. Click "Start Full Scan" button
2. Confirmation dialog: "Start complete 3D scan workflow? Duration: ~3-5 minutes"
3. Status changes to "Starting scan..." (orange)
4. Real-time updates every 2 seconds:
   - "Scanning in progress... [current step]"
5. On completion:
   - "Scan Complete! (X points)" (green)
   - Auto-reset to "Ready to Start" after 5 seconds
6. On failure:
   - "Scan Failed: [error]" (red)
   - Auto-reset to "Ready to Start" after 10 seconds

#### Safety Features
- Prevents multiple simultaneous scans
- Non-blocking subprocess execution
- Automatic process cleanup
- Error recovery with status reset

### 4. Technical Details

#### Files Modified
- `tools/web_dashboard.py`:
  - Added `subprocess` import
  - Added global `scan_process` and `scan_status` variables
  - Implemented `monitor_scan_process()` function
  - Added `startFullScan()` JavaScript function
  - Added `monitorScanProgress()` JavaScript polling
  - Added `/api/scan/start` and `/api/scan/status` routes

#### Process Management
- Subprocess started with `Popen` for non-blocking execution
- Stdout piped and monitored in separate thread
- Process exit code checked for success/failure
- Automatic output directory detection

#### Status Tracking
```python
scan_status = {
    'scan_id': None,
    'status': 'idle',  # idle, running, completed, failed
    'progress': '',
    'error': None,
    'points': 0,
    'output_dir': None
}
```

## Testing

### Manual Test
1. Access dashboard: `http://192.168.0.70:5000`
2. Click "Start Full Scan" button
3. Confirm dialog
4. Watch status updates
5. Verify scan completion and file output in `scans/` directory

### Expected Output
```
scans/
└── YYMMDD-HHMM/
    ├── _lidar.pkl           # Raw LiDAR data
    ├── _pointcloud.ply      # 3D point cloud
    └── _blended_fused.jpg   # Panorama texture (if camera enabled)
```

## Integration with Existing Features

### Works With
- ✅ Automatic stepper motor power management (relay auto-off after scan)
- ✅ Axis inversion configuration (respects INVERT_X/Y/Z settings)
- ✅ Configuration editor (uses current config.json settings)
- ✅ Live LiDAR view (can run simultaneously)
- ✅ Scan history browser (automatically shows new scans)

### Requirements
- Python 3.11+
- Flask 3.0+
- All PiLiDAR dependencies (see requirements.txt)
- Hardware: Pi 5, STL27L LiDAR, stepper motor, relay module

## Usage Examples

### Basic Scan
1. Navigate to dashboard
2. Click "Start Full Scan"
3. Wait 3-5 minutes
4. View results in scan history

### Monitoring via Console
```bash
tail -f /tmp/dashboard.log
# Shows real-time [SCAN] output from PiLiDAR.py
```

### Check Last Scan
```bash
ls -lt scans/ | head -5
```

## Known Limitations
- Only one scan can run at a time (enforced by API)
- Progress updates depend on PiLiDAR.py output format
- Point count extraction requires "X points" string in output
- Camera capture only if `ENABLE_CAM: true` in config

## Future Enhancements
- [ ] Add pause/resume functionality
- [ ] Show estimated time remaining
- [ ] Live preview during scan
- [ ] Scan queue system for multiple scans
- [ ] Email/notification on completion
- [ ] Download scan results as ZIP

## Version
- Implemented: 2025-01-XX
- Dashboard Version: 0.9.1-beta
- Branch: Version0.9.1-beta
