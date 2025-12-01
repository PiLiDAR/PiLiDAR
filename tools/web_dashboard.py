#!/usr/bin/env python3
"""
PiLiDAR Web Dashboard - Central Hub for all web-based tools

Usage:
    python tools/web_dashboard.py [--port PORT]
    
Options:
    --port PORT    Web server port (default: 5000)

This script provides a unified web interface for:
- Live LiDAR visualization with quality metrics
- Configuration editor
- Scan history browser
- System status monitoring

Access from any device at http://<raspberry-pi-ip>:5000
"""

import sys
import os
import argparse
import json
import threading
import time
from collections import deque
from typing import Any, Dict, Optional, Tuple
import copy
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from flask import Flask, render_template_string, jsonify, request, redirect, url_for

from lib.config import Config
from lib.lidar_driver import Lidar


# Main Dashboard HTML
DASHBOARD_HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>PiLiDAR Dashboard</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #eee;
            min-height: 100vh;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 40px 20px;
            text-align: center;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        .header h1 {
            font-size: 48px;
            font-weight: 300;
            letter-spacing: 2px;
            margin-bottom: 10px;
        }
        .header .subtitle {
            font-size: 16px;
            opacity: 0.9;
        }
        .container {
            max-width: 1400px;
            margin: 40px auto;
            padding: 0 20px;
        }
        .tool-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 30px;
            margin-bottom: 40px;
        }
        .tool-card {
            background: #16213e;
            border-radius: 12px;
            padding: 30px;
            box-shadow: 0 8px 16px rgba(0,0,0,0.3);
            transition: all 0.3s ease;
            cursor: pointer;
            border: 2px solid transparent;
        }
        .tool-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 12px 24px rgba(0,0,0,0.4);
            border-color: #667eea;
        }
        .tool-icon {
            font-size: 48px;
            margin-bottom: 20px;
        }
        .tool-title {
            font-size: 24px;
            font-weight: 600;
            margin-bottom: 15px;
            color: #667eea;
        }
        .tool-description {
            font-size: 14px;
            line-height: 1.6;
            color: #8892b0;
            margin-bottom: 20px;
        }
        .tool-status {
            display: inline-block;
            padding: 6px 12px;
            border-radius: 6px;
            font-size: 12px;
            font-weight: 500;
            margin-top: 10px;
        }
        .status-running {
            background: rgba(76, 175, 80, 0.2);
            color: #4CAF50;
            border: 1px solid #4CAF50;
        }
        .status-ready {
            background: rgba(33, 150, 243, 0.2);
            color: #2196F3;
            border: 1px solid #2196F3;
        }
        .status-offline {
            background: rgba(158, 158, 158, 0.2);
            color: #9e9e9e;
            border: 1px solid #9e9e9e;
        }
        .system-info {
            background: #16213e;
            border-radius: 12px;
            padding: 25px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.2);
        }
        .system-info h2 {
            font-size: 20px;
            margin-bottom: 20px;
            color: #667eea;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .info-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
        }
        .info-item {
            text-align: center;
        }
        .info-label {
            font-size: 12px;
            color: #8892b0;
            margin-bottom: 8px;
        }
        .info-value {
            font-size: 24px;
            font-weight: 600;
            color: #fff;
        }
        .footer {
            text-align: center;
            padding: 30px;
            color: #6b7c93;
            font-size: 14px;
        }
        .btn {
            display: inline-block;
            padding: 10px 20px;
            background: #667eea;
            color: white;
            text-decoration: none;
            border-radius: 6px;
            font-weight: 500;
            transition: all 0.3s ease;
            border: none;
            cursor: pointer;
        }
        .btn:hover {
            background: #5568d3;
            transform: translateY(-2px);
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🛰️ PiLiDAR Dashboard</h1>
        <div class="subtitle">Version 0.9.1-beta | Raspberry Pi 5 3D Scanner</div>
    </div>

    <div class="container">
        <div class="tool-grid">
            <!-- Live Visualization -->
            <div class="tool-card" onclick="window.location.href='/lidar'">
                <div class="tool-icon">📡</div>
                <div class="tool-title">Live LiDAR View</div>
                <div class="tool-description">
                    Real-time 2D polar visualization with quality metrics. Monitor point density, 
                    coverage, noise level, and overall scan quality during data acquisition.
                </div>
                <span class="tool-status status-running" id="lidar-status">● Running</span>
            </div>

            <!-- Config Editor -->
            <div class="tool-card" onclick="window.location.href='/config'">
                <div class="tool-icon">⚙️</div>
                <div class="tool-title">Configuration Editor</div>
                <div class="tool-description">
                    Edit scanner settings via web interface. Includes offset calibration wizard, 
                    live JSON validation, and automatic backups.
                </div>
                <span class="tool-status status-ready">Ready</span>
            </div>

            <!-- Scan History -->
            <div class="tool-card" onclick="window.location.href='/scans'">
                <div class="tool-icon">📂</div>
                <div class="tool-title">Scan History</div>
                <div class="tool-description">
                    Browse previous scans, view point clouds, download PLY files, 
                    and manage scan metadata. Export bundles for CloudCompare.
                </div>
                <span class="tool-status status-ready">Ready</span>
            </div>

            <!-- System Monitor -->
            <div class="tool-card" onclick="window.location.href='/system'">
                <div class="tool-icon">💻</div>
                <div class="tool-title">System Monitor</div>
                <div class="tool-description">
                    View CPU temperature, memory usage, disk space, and GPIO status. 
                    Monitor hardware health and performance metrics.
                </div>
                <span class="tool-status status-ready">Ready</span>
            </div>

            <!-- Documentation -->
            <div class="tool-card" onclick="window.open('https://github.com/ORPA1988/PiLiDAR', '_blank')">
                <div class="tool-icon">📚</div>
                <div class="tool-title">Documentation</div>
                <div class="tool-description">
                    Access user manual, API reference, troubleshooting guides, 
                    and hardware assembly instructions on GitHub.
                </div>
                <span class="tool-status status-ready">View Docs</span>
            </div>

            <!-- Quick Actions -->
            <div class="tool-card">
                <div class="tool-icon">⚡</div>
                <div class="tool-title">Quick Actions</div>
                <div class="tool-description" style="margin-bottom: 15px;">
                    Common tasks and utilities for scanner operation.
                </div>
                <button class="btn" onclick="startScan()">Start Full Scan</button>
                <button class="btn" onclick="calibrateOffsets()" style="margin-left: 10px;">Calibrate</button>
            </div>
        </div>

        <!-- System Information Panel -->
        <div class="system-info">
            <h2>System Information</h2>
            <div class="info-grid">
                <div class="info-item">
                    <div class="info-label">Scanner Status</div>
                    <div class="info-value" id="scanner-status">Ready</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Total Scans</div>
                    <div class="info-value" id="total-scans">-</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Uptime</div>
                    <div class="info-value" id="uptime">-</div>
                </div>
                <div class="info-item">
                    <div class="info-label">Version</div>
                    <div class="info-value">0.9.1β</div>
                </div>
            </div>
        </div>
    </div>

    <div class="footer">
        PiLiDAR © 2025 | Raspberry Pi 5 3D Scanner | <a href="https://github.com/ORPA1988/PiLiDAR" style="color: #667eea;">GitHub</a>
    </div>

    <script>
        function startScan() {
            if (confirm('Start a full 3D scan? This will activate the stepper motor and LiDAR.')) {
                alert('Feature coming soon! Use SSH terminal: python PiLiDAR.py');
            }
        }

        function calibrateOffsets() {
            window.location.href = '/config#calibration';
        }

        // Update system info
        async function updateSystemInfo() {
            try {
                const response = await fetch('/api/system');
                const data = await response.json();
                
                if (data.scans_count !== undefined) {
                    document.getElementById('total-scans').textContent = data.scans_count;
                }
                if (data.uptime !== undefined) {
                    document.getElementById('uptime').textContent = data.uptime;
                }
            } catch (error) {
                console.error('Failed to update system info:', error);
            }
        }

        // Update on load and every 30 seconds
        updateSystemInfo();
        setInterval(updateSystemInfo, 30000);
    </script>
</body>
</html>
'''

# Import HTML templates from other modules
# We'll modify the templates to use correct API paths
LIDAR_HTML_TEMPLATE = '''<!DOCTYPE html>
<html>
<head>
    <title>Live LiDAR View</title>
    <script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
        }
        #plot {
            width: 100%;
            height: 85vh;
        }
        .controls {
            margin-bottom: 20px;
            padding: 15px;
            background: #2a2a2a;
            border-radius: 8px;
        }
        .info {
            display: inline-block;
            margin-right: 30px;
            font-size: 14px;
        }
        button {
            padding: 8px 16px;
            margin: 0 5px;
            background: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
        }
        button:hover {
            background: #45a049;
        }
        button.danger {
            background: #f44336;
        }
        .metric {
            text-align: center;
        }
        .metric-label {
            font-size: 11px;
            color: #999;
            margin-bottom: 5px;
        }
        .metric-value {
            font-size: 16px;
            color: #fff;
        }
        #quality-panel {
            background: #2a2a2a;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            display: none;
        }
        #quality-panel h3 {
            margin: 0 0 10px 0;
            font-size: 16px;
        }
    </style>
</head>
<body>
    <div class="controls">
        <a href="/" style="color: #667eea; text-decoration: none; margin-right: 20px; font-weight: 600; display: inline-flex; align-items: center; gap: 5px;">🏠 Dashboard</a>
        <span class="info" title="Total number of LiDAR points received since start">Points: <span id="point-count">0</span></span>
        <span class="info" title="Current vertical angle of the scanner head (stepper motor position)">Z-Angle: <span id="z-angle">0.0</span>°</span>
        <span class="info" title="Number of points currently displayed in the buffer">Buffer: <span id="buffer-size">0</span></span>
        <span class="info" title="Overall scan quality score (0-100) based on density, coverage and noise">Quality: <span id="quality-score" style="font-weight: bold;">0</span>/100</span>
        <span class="info" title="Maximum display distance in meters">Range: <span id="current-range">6.0</span>m</span>
        <label style="margin-left: 15px; margin-right: 5px;" title="Adjust the maximum distance shown in the plot (1-10 meters)">Max Distance:</label>
        <input type="range" id="maxDistanceSlider" min="1" max="10" step="0.5" value="6" 
               style="width: 150px; vertical-align: middle;" 
               oninput="updateMaxDistance(this.value)">
        <button onclick="clearPlot()">Clear</button>
    </div>
    <div id="quality-panel">
        <h3>Scan Quality Metrics</h3>
        <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px;">
            <div class="metric">
                <div class="metric-label" title="Average number of LiDAR points per degree of angular coverage">Point Density</div>
                <div class="metric-value"><span id="metric-density">0</span> pts/deg</div>
            </div>
            <div class="metric">
                <div class="metric-label" title="Percentage of the 180° scan range that contains valid measurements">Coverage</div>
                <div class="metric-value"><span id="metric-coverage">0</span>%</div>
            </div>
            <div class="metric">
                <div class="metric-label" title="Standard deviation of distance measurements - lower is better (indicates less noise)">Noise Level</div>
                <div class="metric-value"><span id="metric-noise">0</span> mm</div>
            </div>
            <div class="metric">
                <div class="metric-label" title="Average signal intensity of valid measurements (0-255)">Avg Intensity</div>
                <div class="metric-value"><span id="metric-intensity">0</span></div>
            </div>
            <div class="metric">
                <div class="metric-label" title="Number of valid points within the max distance range">Valid Points</div>
                <div class="metric-value"><span id="metric-valid">0</span></div>
            </div>
            <div class="metric">
                <div class="metric-label" title="Overall quality score: 0.4×density + 0.4×coverage + 0.2×noise_inverse">Quality Score</div>
                <div class="metric-value" id="quality-indicator" style="font-weight: bold; font-size: 18px;"><span id="metric-quality">0</span>/100</div>
            </div>
        </div>
    </div>
    <div id="plot"></div>
    <script>
        let maxDistance = {{ max_distance }};
        
        // Coordinate system annotations (arrows showing X/Y axes)
        const coordSystemAnnotations = [
            // X-axis arrow (0° direction, right side)
            {
                x: 1.15,
                y: 0.5,
                xref: 'paper',
                yref: 'paper',
                text: '<b>+X →</b>',
                showarrow: false,
                font: {size: 14, color: '#ff6b6b'}
            },
            // Y-axis arrow (90° direction, top)
            {
                x: 0.5,
                y: 1.05,
                xref: 'paper',
                yref: 'paper',
                text: '<b>+Y ↑</b>',
                showarrow: false,
                font: {size: 14, color: '#4ecdc4'}
            },
            // -X direction (180°, left)
            {
                x: -0.15,
                y: 0.5,
                xref: 'paper',
                yref: 'paper',
                text: '<b>← -X</b>',
                showarrow: false,
                font: {size: 14, color: '#ff6b6b'}
            },
            // -Y direction (270°, bottom)
            {
                x: 0.5,
                y: -0.05,
                xref: 'paper',
                yref: 'paper',
                text: '<b>↓ -Y</b>',
                showarrow: false,
                font: {size: 14, color: '#4ecdc4'}
            }
        ];
        
        const data = [{
            type: 'scatterpolar',
            mode: 'markers',
            r: [],
            theta: [],
            marker: {
                color: [],
                size: 3,
                colorscale: 'Viridis',
                showscale: true,
                colorbar: {
                    title: 'Intensity',
                    thickness: 15
                }
            }
        }];
        
        const layout = {
            polar: {
                radialaxis: {
                    visible: true,
                    range: [0, maxDistance * 1000],
                    title: {
                        text: 'Distance (mm)',
                        font: {size: 12}
                    }
                },
                angularaxis: {
                    direction: 'clockwise',
                    rotation: 90,
                    tickmode: 'linear',
                    dtick: 30
                }
            },
            showlegend: false,
            title: `Live LiDAR Scan (2D Polar View) - Max Range: ${maxDistance.toFixed(1)}m`,
            paper_bgcolor: '#1a1a1a',
            plot_bgcolor: '#1a1a1a',
            font: {color: '#fff'},
            annotations: coordSystemAnnotations
        };
        
        Plotly.newPlot('plot', data, layout, {responsive: true});
        
        // Update max distance from slider
        function updateMaxDistance(value) {
            maxDistance = parseFloat(value);
            document.getElementById('current-range').textContent = maxDistance.toFixed(1);
            layout.polar.radialaxis.range = [0, maxDistance * 1000];
            layout.title = `Live LiDAR Scan (2D Polar View) - Max Range: ${maxDistance.toFixed(1)}m`;
            Plotly.relayout('plot', layout);
        }
        
        async function updatePlot() {
            try {
                const response = await fetch('/api/lidar/data');
                const jsonData = await response.json();
                
                if (jsonData.angles.length > 0) {
                    data[0].r = jsonData.distances;
                    data[0].theta = jsonData.angles;
                    data[0].marker.color = jsonData.intensities;
                    Plotly.update('plot', data, layout);
                }
                
                document.getElementById('point-count').textContent = jsonData.point_count;
                document.getElementById('z-angle').textContent = jsonData.z_angle.toFixed(2);
                document.getElementById('buffer-size').textContent = jsonData.buffer_size;
                
                const quality = jsonData.quality;
                const qualityScore = Math.round(quality.quality_score);
                document.getElementById('quality-score').textContent = qualityScore;
                
                if (jsonData.buffer_size > 100) {
                    document.getElementById('quality-panel').style.display = 'block';
                    document.getElementById('metric-density').textContent = quality.point_density.toFixed(1);
                    document.getElementById('metric-coverage').textContent = quality.coverage_percent.toFixed(1);
                    document.getElementById('metric-noise').textContent = quality.noise_level.toFixed(1);
                    document.getElementById('metric-intensity').textContent = Math.round(quality.avg_intensity);
                    document.getElementById('metric-valid').textContent = quality.valid_points;
                    document.getElementById('metric-quality').textContent = qualityScore;
                    
                    const indicator = document.getElementById('quality-indicator');
                    if (qualityScore >= 80) {
                        indicator.style.color = '#4CAF50';
                    } else if (qualityScore >= 60) {
                        indicator.style.color = '#ff9800';
                    } else {
                        indicator.style.color = '#f44336';
                    }
                }
            } catch (error) {
                console.error('Error:', error);
            }
        }
        
        function clearPlot() {
            fetch('/api/lidar/clear', {method: 'POST'});
        }
        
        setInterval(updatePlot, 50);
    </script>
</body>
</html>
'''


class WebLidarView:
    """Web-based LiDAR visualization (integrated into dashboard)."""
    
    def __init__(self, max_distance=6000, max_points=5000):
        self.max_distance = max_distance
        self.max_points = max_points
        self.angles = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.intensities = deque(maxlen=max_points)
        self.point_count = 0
        self.current_z_angle = 0.0
        self.lock = threading.Lock()
        self.running = True
        
        self.quality_metrics = {
            'point_density': 0.0,
            'coverage_percent': 0.0,
            'noise_level': 0.0,
            'avg_intensity': 0.0,
            'valid_points': 0,
            'quality_score': 0.0
        }
        
    def add_point(self, angle_deg, distance_mm, intensity=None):
        if distance_mm > 0:
            with self.lock:
                self.angles.append(angle_deg)
                self.distances.append(distance_mm)
                self.intensities.append(intensity if intensity is not None else 128)
                self.point_count += 1
                # Recalculate quality metrics periodically (every 50 points)
                if self.point_count % 50 == 0:
                    self._calculate_quality_metrics()
    
    def update_z_angle(self, z_angle):
        with self.lock:
            self.current_z_angle = z_angle
            self._calculate_quality_metrics()
    
    def _calculate_quality_metrics(self):
        if len(self.distances) < 10:
            return
        
        angles_arr = np.array(list(self.angles))
        distances_arr = np.array(list(self.distances))
        intensities_arr = np.array(list(self.intensities))
        
        # Use max_distance in mm for filtering
        valid_mask = (distances_arr > 0) & (distances_arr < self.max_distance)
        valid_distances = distances_arr[valid_mask]
        valid_intensities = intensities_arr[valid_mask]
        valid_angles = angles_arr[valid_mask]
        
        self.quality_metrics['valid_points'] = int(np.sum(valid_mask))
        
        if len(valid_distances) == 0:
            return
        
        # Calculate point density (points per degree)
        angle_range = np.max(valid_angles) - np.min(valid_angles)
        if angle_range > 0:
            self.quality_metrics['point_density'] = float(len(valid_distances) / angle_range)
        
        # Calculate coverage percentage (how many 1-degree bins have data)
        bins = np.arange(0, 181, 1)
        hist, _ = np.histogram(valid_angles, bins=bins)
        covered_bins = np.sum(hist > 0)
        self.quality_metrics['coverage_percent'] = float((covered_bins / 180) * 100)
        
        # Noise level (standard deviation of distances)
        self.quality_metrics['noise_level'] = float(np.std(valid_distances))
        
        # Average intensity
        self.quality_metrics['avg_intensity'] = float(np.mean(valid_intensities))
        
        # Calculate overall quality score (0-100)
        density_score = min(100, (self.quality_metrics['point_density'] / 50) * 100)
        coverage_score = self.quality_metrics['coverage_percent']
        noise_score = max(0, 100 - (self.quality_metrics['noise_level'] / 10))
        
        self.quality_metrics['quality_score'] = float(
            0.4 * density_score + 0.4 * coverage_score + 0.2 * noise_score
        )
    
    def get_data(self):
        with self.lock:
            return {
                'angles': [float(a) for a in self.angles],
                'distances': [float(d) for d in self.distances],
                'intensities': [int(i) for i in self.intensities],
                'point_count': int(self.point_count),
                'buffer_size': len(self.angles),
                'z_angle': float(self.current_z_angle),
                'quality': {
                    'point_density': float(self.quality_metrics['point_density']),
                    'coverage_percent': float(self.quality_metrics['coverage_percent']),
                    'noise_level': float(self.quality_metrics['noise_level']),
                    'avg_intensity': float(self.quality_metrics['avg_intensity']),
                    'valid_points': int(self.quality_metrics['valid_points']),
                    'quality_score': float(self.quality_metrics['quality_score'])
                }
            }
    
    def clear(self):
        with self.lock:
            self.angles.clear()
            self.distances.clear()
            self.intensities.clear()
            self.point_count = 0


class ConfigEditor:
    """Configuration editor (integrated into dashboard)."""
    
    def __init__(self, config_path: str):
        self.config_path = os.path.abspath(config_path)
        self.config: Dict[str, Any] = {}
        self.load_config()
    
    def load_config(self) -> None:
        try:
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            self.config = {}
    
    def save_config(self, new_config: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        try:
            json_str = json.dumps(new_config, indent=4)
            backup_path = self.config_path + '.backup'
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    with open(backup_path, 'w') as b:
                        b.write(f.read())
            with open(self.config_path, 'w') as f:
                f.write(json_str)
            self.config = new_config
            return True, None
        except Exception as e:
            return False, str(e)
    
    def get_config(self) -> Dict[str, Any]:
        return copy.deepcopy(self.config)


def create_app(web_view: WebLidarView, config_editor: ConfigEditor, config_path: str) -> Flask:
    """Create unified Flask application."""
    app = Flask(__name__)
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
    
    @app.route('/')
    def dashboard():
        """Main dashboard."""
        return render_template_string(DASHBOARD_HTML)
    
    @app.route('/lidar')
    def lidar_view():
        """LiDAR visualization page."""
        return render_template_string(LIDAR_HTML_TEMPLATE, max_distance=6.0)
    
    @app.route('/config')
    def config_view():
        """Configuration editor page."""
        # Import config editor HTML here
        from tools.config_editor import HTML_TEMPLATE as CONFIG_HTML
        return render_template_string(CONFIG_HTML)
    
    @app.route('/scans')
    def scans_view():
        """Scan history browser."""
        scans_root = os.path.join(os.path.dirname(config_path), 'scans')
        scan_dirs = []
        if os.path.exists(scans_root):
            scan_dirs = [d for d in os.listdir(scans_root) if os.path.isdir(os.path.join(scans_root, d))]
            scan_dirs.sort(reverse=True)
        
        html = f'''
        <!DOCTYPE html>
        <html>
        <head><title>Scan History</title>
        <style>
            body {{ font-family: Arial; background: #1a1a2e; color: #eee; padding: 20px; }}
            .scan-item {{ background: #16213e; padding: 15px; margin: 10px 0; border-radius: 8px; }}
            a {{ color: #667eea; text-decoration: none; }}
        </style>
        </head>
        <body>
            <h1>📂 Scan History</h1>
            <p><a href="/" style="color: #667eea; text-decoration: none; font-weight: 600; display: inline-flex; align-items: center; gap: 5px;">🏠 Dashboard</a></p>
            <div>
                {"".join(f'<div class="scan-item">{d}</div>' for d in scan_dirs) if scan_dirs else "<p>No scans found</p>"}
            </div>
        </body>
        </html>
        '''
        return html
    
    @app.route('/system')
    def system_view():
        """System monitor."""
        html = '''
        <!DOCTYPE html>
        <html>
        <head><title>System Monitor</title>
        <style>
            body { font-family: Arial; background: #1a1a2e; color: #eee; padding: 20px; }
            .metric { background: #16213e; padding: 20px; margin: 10px 0; border-radius: 8px; }
            a { color: #667eea; text-decoration: none; }
        </style>
        </head>
        <body>
            <h1>💻 System Monitor</h1>
            <p><a href="/" style="color: #667eea; text-decoration: none; font-weight: 600; display: inline-flex; align-items: center; gap: 5px;">🏠 Dashboard</a></p>
            <div class="metric">
                <h3>CPU Temperature</h3>
                <p id="cpu-temp">Loading...</p>
            </div>
            <div class="metric">
                <h3>Memory Usage</h3>
                <p id="memory">Loading...</p>
            </div>
            <script>
                fetch('/api/system').then(r => r.json()).then(d => {
                    document.getElementById('cpu-temp').textContent = d.cpu_temp || 'N/A';
                    document.getElementById('memory').textContent = d.memory || 'N/A';
                });
            </script>
        </body>
        </html>
        '''
        return html
    
    # API Endpoints
    @app.route('/api/lidar/data')
    def get_lidar_data():
        return jsonify(web_view.get_data())
    
    @app.route('/api/lidar/clear', methods=['POST'])
    def clear_lidar():
        web_view.clear()
        return jsonify({'status': 'ok'})
    
    @app.route('/api/config', methods=['GET'])
    def get_config():
        return jsonify({'success': True, 'config': config_editor.get_config()})
    
    @app.route('/api/config', methods=['POST'])
    def save_config():
        try:
            data = request.get_json()
            if not data or 'config' not in data:
                return jsonify({'success': False, 'error': 'Missing config data'}), 400
            success, error = config_editor.save_config(data['config'])
            if success:
                return jsonify({'success': True})
            else:
                return jsonify({'success': False, 'error': error}), 500
        except Exception as e:
            return jsonify({'success': False, 'error': str(e)}), 500
    
    @app.route('/api/system')
    def get_system_info():
        scans_root = os.path.join(os.path.dirname(config_path), 'scans')
        scans_count = 0
        if os.path.exists(scans_root):
            scans_count = len([d for d in os.listdir(scans_root) if os.path.isdir(os.path.join(scans_root, d))])
        
        try:
            with open('/proc/uptime', 'r') as f:
                uptime_seconds = float(f.read().split()[0])
                uptime = f"{int(uptime_seconds // 3600)}h {int((uptime_seconds % 3600) // 60)}m"
        except:
            uptime = "N/A"
        
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                cpu_temp = f"{float(f.read()) / 1000:.1f}°C"
        except:
            cpu_temp = "N/A"
        
        return jsonify({
            'scans_count': scans_count,
            'uptime': uptime,
            'cpu_temp': cpu_temp,
            'memory': 'N/A'
        })
    
    @app.route('/api/stop', methods=['POST'])
    def stop_server():
        web_view.running = False
        threading.Thread(target=lambda: (time.sleep(0.5), os._exit(0))).start()
        return jsonify({'status': 'stopping'})
    
    return app


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='PiLiDAR Web Dashboard')
    parser.add_argument('--port', type=int, default=5000, help='Web server port (default: 5000)')
    parser.add_argument('--max-distance', type=float, default=6.0, help='Max LiDAR range in meters (default: 6.0)')
    parser.add_argument('--config', type=str, default='config.json', help='Path to config.json')
    args = parser.parse_args()
    
    # Resolve config path
    if not os.path.isabs(args.config):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', args.config)
    else:
        config_path = args.config
    
    print("Starting PiLiDAR Web Dashboard...")
    print(f"Config file: {config_path}")
    
    # Disable GPIO in Config for live view
    import lib.config as config_module
    original_gpio_setup = config_module.Config.gpio_setup
    config_module.Config.gpio_setup = lambda self, debug=False: None
    
    try:
        config = Config()
        config.relay_device = None
        config.init(scan_id="_live")
    finally:
        config_module.Config.gpio_setup = original_gpio_setup
    
    # Initialize components
    max_dist = args.max_distance * 1000
    web_view = WebLidarView(max_distance=max_dist, max_points=10000)
    config_editor = ConfigEditor(config_path)
    
    # Initialize LiDAR
    print(f"Connecting to LiDAR on {config.PORT}...")
    lidar = Lidar(config, visualization=None)
    
    # LiDAR reading threads
    def lidar_read_thread():
        try:
            lidar.read_loop(callback=None, max_packages=None)
        except Exception as e:
            print(f"LiDAR read error: {e}")
    
    def lidar_monitor_thread():
        last_index = 0
        while web_view.running:
            try:
                current_index = lidar.out_i * lidar.dlength
                if current_index < last_index:
                    last_index = 0
                if current_index > last_index:
                    new_points = lidar.points_2d[last_index:current_index]
                    for point in new_points:
                        x, y, intensity = point
                        distance = np.sqrt(x**2 + y**2)
                        angle = np.arctan2(y, x) * 180 / np.pi
                        if angle < 0:
                            angle += 360
                        web_view.add_point(angle, distance, int(intensity))
                    if lidar.z_angle is not None:
                        web_view.update_z_angle(lidar.z_angle)
                    last_index = current_index
                time.sleep(0.01)
            except Exception as e:
                print(f"Monitor error: {e}")
                time.sleep(0.1)
    
    # Start background threads
    threading.Thread(target=lidar_read_thread, daemon=True).start()
    threading.Thread(target=lidar_monitor_thread, daemon=True).start()
    
    # Create and run Flask app
    app = create_app(web_view, config_editor, config_path)
    
    print(f"\n{'='*60}")
    print(f"  🛰️  PiLiDAR Dashboard Ready!")
    print(f"  Access from any device at: http://192.168.0.70:{args.port}")
    print(f"  Or locally at: http://localhost:{args.port}")
    print(f"{'='*60}\n")
    print("Press Ctrl+C to stop...")
    
    app.run(host='0.0.0.0', port=args.port, debug=False)


if __name__ == "__main__":
    main()
