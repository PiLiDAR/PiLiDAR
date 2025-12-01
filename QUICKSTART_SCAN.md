# Quick Start: Full Scan Automation

## Access Dashboard
```
http://192.168.0.70:5000
```

## Start a Scan

### Via Web Interface (Recommended)
1. Click **"🚀 Start Full Scan"** button
2. Confirm: "Start complete 3D scan workflow?"
3. Wait for completion (~3-5 minutes)
4. Status updates automatically every 2 seconds

### Via API (Advanced)
```bash
# Start scan
curl -X POST http://192.168.0.70:5000/api/scan/start

# Check status
curl http://192.168.0.70:5000/api/scan/status
```

## Status Indicators

| Color  | Status                    | Meaning                          |
|--------|---------------------------|----------------------------------|
| 🟢 Green | "Ready to Start"         | System ready, no scan running    |
| 🟠 Orange | "Scanning in progress..." | Scan currently executing         |
| 🟢 Green | "Scan Complete! (X pts)"  | Scan finished successfully       |
| 🔴 Red   | "Scan Failed: [error]"   | Scan encountered error          |

## Scan Workflow Steps

1. **Starting scan...** (2-5 seconds)
   - Process initialization

2. **Camera panorama capture...** (30-60 seconds, if ENABLE_CAM=true)
   - 4 HDR photos at 90° intervals
   - Auto-exposure bracketing

3. **LiDAR 180° scan...** (60-90 seconds)
   - Stepper motor rotation
   - 21,600 samples/second
   - ~200k-600k points captured

4. **Stitching panorama...** (20-40 seconds, if camera enabled)
   - Hugin panorama fusion
   - HDR blending

5. **3D point cloud processing...** (10-30 seconds)
   - NumPy coordinate transformation
   - Outlier filtering
   - Texture mapping (if panorama available)

6. **Saving results...** (5-10 seconds)
   - PLY file export
   - Pickle data archival

## Output Files

```
scans/YYMMDD-HHMM/
├── _lidar.pkl              # Raw scan data (Python pickle)
├── _pointcloud.ply         # 3D point cloud (CloudCompare)
└── _blended_fused.jpg      # Panorama texture (if camera)
```

## Troubleshooting

### Button does nothing
- Check browser console (F12) for JavaScript errors
- Verify dashboard is running: `pgrep -f web_dashboard`

### "Scan already in progress" error
- Wait for current scan to complete
- Force reset: Restart dashboard

### Scan stuck at "Starting scan..."
- Check logs: `tail -f /tmp/dashboard.log`
- Verify hardware connections (LiDAR, stepper, relay)

### "Scan Failed" status
- Read error message in status display
- Check `/tmp/dashboard.log` for details
- Verify config.json settings
- Ensure no GPIO conflicts

### No output files
- Check `scans/` directory permissions
- Verify disk space: `df -h`
- Review PiLiDAR.py logs in dashboard

## Advanced Usage

### Monitor Scan Progress
```bash
# Watch dashboard logs
tail -f /tmp/dashboard.log

# Check scan output directory
watch -n 1 'ls -lh scans/$(ls -t scans/ | head -1)'
```

### Manual Scan (Without Web Interface)
```bash
cd /home/pi/PilidarPi5/PiLiDAR
python3 PiLiDAR.py
```

### Restart Dashboard
```bash
pkill -f web_dashboard && sleep 2 && \
python3 tools/web_dashboard.py --port 5000 > /tmp/dashboard.log 2>&1 &
```

## Configuration Tips

### Fast Scan Mode
```json
{
  "ENABLE_CAM": false,           // Skip camera capture
  "ENABLE_FILTERING": false,     // Skip statistical filtering
  "TARGET_RES": "1/6"           // Lower angular resolution
}
```

### High Quality Mode
```json
{
  "ENABLE_CAM": true,            // Include panorama texture
  "IMGCOUNT": 16,                // More photos (better quality)
  "ENABLE_FILTERING": true,      // Statistical outlier removal
  "TARGET_RES": "1/12"          // Higher angular resolution
}
```

### Stepper Motor Tuning
```json
{
  "STEP_DELAY": 0.0005,          // Motor speed (lower=faster)
  "IDLE_OFF_DELAY": 10           // Auto power-off delay (seconds)
}
```

## Safety Notes

⚠️ **Before Scanning:**
- Ensure clear 360° view around scanner
- Remove objects within 0.3m of LiDAR sensor
- Verify stepper motor is free to rotate 180°
- Check relay connections if using auto power-off

⚠️ **During Scan:**
- Do not move scanner
- Avoid bright light changes (if camera enabled)
- Do not disconnect power
- Do not interrupt via SSH

⚠️ **After Scan:**
- Motor automatically powers off after 10 seconds
- Safe to disconnect after "Scan Complete" status
- Backup important scans immediately

## Performance Expectations

| Configuration      | Duration | Points   | File Size |
|--------------------|----------|----------|-----------|
| LiDAR only         | ~2 min   | 200k-400k| 5-15 MB   |
| LiDAR + Camera (4) | ~3 min   | 200k-400k| 8-20 MB   |
| LiDAR + Camera (16)| ~5 min   | 200k-400k| 15-30 MB  |
| With filtering     | +30 sec  | -10%     | -10%      |

## Support

- Documentation: `/home/pi/PilidarPi5/PiLiDAR/README.md`
- Configuration: `http://192.168.0.70:5000/config`
- Scan History: `http://192.168.0.70:5000/scans`
- System Status: `http://192.168.0.70:5000/system`
