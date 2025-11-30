#!/usr/bin/env python3
"""
Web-based Live 2D Polar Plot Visualization for LiDAR Scanner

Usage:
    python tools/live_view_web.py [--port PORT] [--max-distance METERS]
    
Options:
    --port PORT              Web server port (default: 5000)
    --max-distance METERS    Maximum display range in meters (default: 6.0)

This script visualizes LiDAR data in real-time via a web interface.
Access from any device on the network at http://<raspberry-pi-ip>:5000
"""

import sys
import os
import argparse
import json
import threading
import time
from collections import deque
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from flask import Flask, render_template_string, jsonify

from lib.config import Config
from lib.lidar_driver import Lidar


# HTML template with embedded JavaScript for live plotting
HTML_TEMPLATE = '''
<!DOCTYPE html>
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
        button.danger:hover {
            background: #da190b;
        }
        .slider-container {
            display: inline-block;
            margin: 0 10px;
        }
        input[type="range"] {
            width: 200px;
            vertical-align: middle;
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
        #quality-panel h3 {
            cursor: pointer;
            user-select: none;
        }
        #quality-panel h3:hover {
            color: #4CAF50;
        }
    </style>
</head>
<body>
    <div class="controls">
        <span class="info">Points: <span id="point-count">0</span></span>
        <span class="info">Z-Angle: <span id="z-angle">0.0</span>°</span>
        <span class="info">Buffer: <span id="buffer-size">0</span></span>
        <span class="info">Quality: <span id="quality-score" style="font-weight: bold;">0</span>/100</span>
        <button onclick="clearPlot()">Clear</button>
        <div class="slider-container">
            <label>Max Range: <span id="range-value">{{ max_distance }}</span>m</label>
            <input type="range" id="range-slider" min="1" max="20" step="0.5" value="{{ max_distance }}" 
                   oninput="updateRange(this.value)">
        </div>
        <button class="danger" onclick="stopServer()">Stop Server</button>
    </div>
    <div id="quality-panel" style="background: #2a2a2a; padding: 15px; border-radius: 8px; margin-bottom: 20px; display: none;">
        <h3 style="margin: 0 0 10px 0; font-size: 16px;">Scan Quality Metrics</h3>
        <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px;">
            <div class="metric">
                <div class="metric-label">Point Density</div>
                <div class="metric-value"><span id="metric-density">0</span> pts/deg</div>
            </div>
            <div class="metric">
                <div class="metric-label">Coverage</div>
                <div class="metric-value"><span id="metric-coverage">0</span>%</div>
            </div>
            <div class="metric">
                <div class="metric-label">Noise Level</div>
                <div class="metric-value"><span id="metric-noise">0</span> mm</div>
            </div>
            <div class="metric">
                <div class="metric-label">Avg Intensity</div>
                <div class="metric-value"><span id="metric-intensity">0</span></div>
            </div>
            <div class="metric">
                <div class="metric-label">Valid Points</div>
                <div class="metric-value"><span id="metric-valid">0</span></div>
            </div>
            <div class="metric">
                <div class="metric-label">Quality Score</div>
                <div class="metric-value" id="quality-indicator" style="font-weight: bold; font-size: 18px;"><span id="metric-quality">0</span>/100</div>
            </div>
        </div>
    </div>
    <div id="plot"></div>

    <script>
        let maxDistance = {{ max_distance }};
        let pointCount = 0;
        let bufferSize = 0;
        let zAngle = 0.0;
        
        // Initialize polar plot
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
                cmin: 0,
                cmax: 255
            }
        }];
        
        const layout = {
            polar: {
                radialaxis: {
                    visible: true,
                    range: [0, maxDistance * 1000]  // Convert to mm
                },
                angularaxis: {
                    direction: 'clockwise',
                    rotation: 90
                }
            },
            showlegend: false,
            title: 'Live LiDAR Scan (2D Polar View)',
            paper_bgcolor: '#1a1a1a',
            plot_bgcolor: '#1a1a1a',
            font: {
                color: '#fff'
            }
        };
        
        Plotly.newPlot('plot', data, layout, {responsive: true});
        
        // Fetch and update data
        async function updatePlot() {
            try {
                const response = await fetch('/data');
                const jsonData = await response.json();
                
                if (jsonData.angles.length > 0) {
                    data[0].r = jsonData.distances;
                    data[0].theta = jsonData.angles;
                    data[0].marker.color = jsonData.intensities;
                    
                    Plotly.update('plot', data, layout);
                }
                
                // Update info display
                pointCount = jsonData.point_count;
                bufferSize = jsonData.buffer_size;
                zAngle = jsonData.z_angle;
                
                document.getElementById('point-count').textContent = pointCount;
                document.getElementById('z-angle').textContent = zAngle.toFixed(2);
                document.getElementById('buffer-size').textContent = bufferSize;
                
                // Update quality metrics
                const quality = jsonData.quality;
                const qualityScore = Math.round(quality.quality_score);
                document.getElementById('quality-score').textContent = qualityScore;
                
                // Show quality panel when data is available
                if (bufferSize > 100) {
                    document.getElementById('quality-panel').style.display = 'block';
                    document.getElementById('metric-density').textContent = quality.point_density.toFixed(1);
                    document.getElementById('metric-coverage').textContent = quality.coverage_percent.toFixed(1);
                    document.getElementById('metric-noise').textContent = quality.noise_level.toFixed(1);
                    document.getElementById('metric-intensity').textContent = Math.round(quality.avg_intensity);
                    document.getElementById('metric-valid').textContent = quality.valid_points;
                    document.getElementById('metric-quality').textContent = qualityScore;
                    
                    // Color code quality score
                    const indicator = document.getElementById('quality-indicator');
                    if (qualityScore >= 80) {
                        indicator.style.color = '#4CAF50';  // Green
                    } else if (qualityScore >= 60) {
                        indicator.style.color = '#ff9800';  // Orange
                    } else {
                        indicator.style.color = '#f44336';  // Red
                    }
                }
                
            } catch (error) {
                console.error('Update error:', error);
            }
        }
        
        function updateRange(value) {
            maxDistance = parseFloat(value);
            document.getElementById('range-value').textContent = value;
            layout.polar.radialaxis.range = [0, maxDistance * 1000];
            Plotly.relayout('plot', layout);
        }
        
        async function clearPlot() {
            await fetch('/clear', {method: 'POST'});
            data[0].r = [];
            data[0].theta = [];
            data[0].marker.color = [];
            Plotly.update('plot', data, layout);
        }
        
        async function stopServer() {
            if (confirm('Stop the LiDAR server?')) {
                await fetch('/stop', {method: 'POST'});
                alert('Server stopping...');
            }
        }
        
        // Update every 50ms (20 FPS)
        setInterval(updatePlot, 50);
    </script>
</body>
</html>
'''


class WebLidarView:
    """Web-based LiDAR visualization using Flask with quality metrics."""
    
    def __init__(self, max_distance=6000, max_points=5000):
        self.max_distance = max_distance
        self.max_points = max_points
        
        # Data storage with circular buffer
        self.angles = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.intensities = deque(maxlen=max_points)
        
        self.point_count = 0
        self.current_z_angle = 0.0
        self.lock = threading.Lock()
        self.running = True
        
        # Quality metrics
        self.quality_metrics = {
            'point_density': 0.0,      # points per degree
            'coverage_percent': 0.0,    # percentage of 180° covered
            'noise_level': 0.0,         # std dev of distances
            'avg_intensity': 0.0,       # mean intensity
            'valid_points': 0,          # points with distance > 0
            'quality_score': 0.0        # overall 0-100 score
        }
        
    def add_point(self, angle_deg, distance_mm, intensity=None):
        """Add a new LiDAR measurement point."""
        if distance_mm > 0:
            with self.lock:
                self.angles.append(angle_deg)
                self.distances.append(distance_mm)
                self.intensities.append(intensity if intensity is not None else 128)
                self.point_count += 1
    
    def update_z_angle(self, z_angle):
        """Update current z-axis rotation angle."""
        with self.lock:
            self.current_z_angle = z_angle
            self._calculate_quality_metrics()
    
    def _calculate_quality_metrics(self):
        """Calculate scan quality metrics from current buffer."""
        if len(self.distances) < 10:
            return
        
        # Convert to numpy for faster computation
        angles_arr = np.array(list(self.angles))
        distances_arr = np.array(list(self.distances))
        intensities_arr = np.array(list(self.intensities))
        
        # Valid points (distance > 0 and < max_distance)
        valid_mask = (distances_arr > 0) & (distances_arr < self.max_distance)
        valid_distances = distances_arr[valid_mask]
        valid_intensities = intensities_arr[valid_mask]
        
        self.quality_metrics['valid_points'] = int(np.sum(valid_mask))
        
        if len(valid_distances) == 0:
            return
        
        # Point density: points per degree
        angle_range = np.max(angles_arr) - np.min(angles_arr)
        if angle_range > 0:
            self.quality_metrics['point_density'] = float(len(valid_distances) / angle_range)
        
        # Coverage: percentage of 180° covered with data
        # Discretize into 1° bins
        bins = np.arange(0, 181, 1)
        hist, _ = np.histogram(angles_arr[valid_mask], bins=bins)
        covered_bins = np.sum(hist > 0)
        self.quality_metrics['coverage_percent'] = float((covered_bins / 180) * 100)
        
        # Noise level: standard deviation of distances (lower is better)
        self.quality_metrics['noise_level'] = float(np.std(valid_distances))
        
        # Average intensity
        self.quality_metrics['avg_intensity'] = float(np.mean(valid_intensities))
        
        # Quality score (0-100)
        # Factors: density (40%), coverage (40%), low noise (20%)
        density_score = min(100, (self.quality_metrics['point_density'] / 50) * 100)  # 50 pts/deg = 100%
        coverage_score = self.quality_metrics['coverage_percent']
        noise_score = max(0, 100 - (self.quality_metrics['noise_level'] / 10))  # <100mm noise = 90%+
        
        self.quality_metrics['quality_score'] = float(
            0.4 * density_score + 0.4 * coverage_score + 0.2 * noise_score
        )
    
    def get_data(self):
        """Get current data for web display."""
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
        """Clear all data."""
        with self.lock:
            self.angles.clear()
            self.distances.clear()
            self.intensities.clear()
            self.point_count = 0


def main():
    """Main function to run web-based LiDAR visualization."""
    parser = argparse.ArgumentParser(description='Web-based Live 2D Polar LiDAR Visualization')
    parser.add_argument('--port', type=int, default=5000,
                       help='Web server port (default: 5000)')
    parser.add_argument('--max-distance', type=float, default=6.0,
                       help='Maximum display range in meters (default: 6.0)')
    args = parser.parse_args()
    
    print("Starting Web-based Live LiDAR View...")
    
    # Temporarily disable GPIO setup in Config
    import lib.config as config_module
    original_gpio_setup = config_module.Config.gpio_setup
    config_module.Config.gpio_setup = lambda self, debug=False: None
    
    try:
        config = Config()
        config.relay_device = None
        config.init(scan_id="_live")
    finally:
        config_module.Config.gpio_setup = original_gpio_setup
    
    # Create web view
    max_dist = args.max_distance * 1000
    web_view = WebLidarView(max_distance=max_dist, max_points=10000)
    
    # Initialize LiDAR
    print(f"Connecting to LiDAR on {config.PORT}...")
    lidar = Lidar(config, visualization=None)
    
    # Setup Flask app
    app = Flask(__name__)
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0
    
    @app.route('/')
    def index():
        return render_template_string(HTML_TEMPLATE, max_distance=args.max_distance)
    
    @app.route('/data')
    def get_data():
        return jsonify(web_view.get_data())
    
    @app.route('/clear', methods=['POST'])
    def clear_data():
        web_view.clear()
        return jsonify({'status': 'ok'})
    
    @app.route('/stop', methods=['POST'])
    def stop_server():
        web_view.running = False
        threading.Thread(target=lambda: (time.sleep(0.5), os._exit(0))).start()
        return jsonify({'status': 'stopping'})
    
    # Start LiDAR reading in two background threads
    # Thread 1: Read serial data continuously
    def lidar_read_thread():
        """Continuously read from serial port and fill buffer."""
        try:
            print("Starting LiDAR serial reader...")
            lidar.read_loop(callback=None, max_packages=None)
        except Exception as e:
            print(f"LiDAR read error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            try:
                lidar.close()
            except:
                pass
    
    # Thread 2: Monitor buffer and update visualization
    def lidar_monitor_thread():
        """Monitor LiDAR buffer and update web view."""
        print("Starting LiDAR monitor thread...")
        last_index = 0
        update_count = 0
        
        while web_view.running:
            try:
                # Get current buffer position
                current_index = lidar.out_i * lidar.dlength
                
                # Handle wrap-around
                if current_index < last_index:
                    last_index = 0
                
                # Process new data
                if current_index > last_index:
                    new_points = lidar.points_2d[last_index:current_index]
                    
                    points_added = 0
                    for x, y, intensity in new_points:
                        if x != 0 or y != 0:
                            distance = np.sqrt(x*x + y*y)
                            if 0 < distance <= web_view.max_distance:
                                angle = np.degrees(np.arctan2(y, x))
                                if angle < 0:
                                    angle += 360
                                web_view.add_point(angle, distance, int(intensity))
                                points_added += 1
                    
                    if points_added > 0:
                        update_count += 1
                        if update_count % 100 == 0:
                            print(f"Updates: {update_count}, Points buffered: {len(web_view.angles)}")
                    
                    last_index = current_index
                
                # Update z-angle
                if lidar.z_angle is not None:
                    web_view.update_z_angle(lidar.z_angle)
                
                # Small sleep to prevent CPU overload
                time.sleep(0.01)
                
            except Exception as e:
                print(f"Monitor thread error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
        
        print("Monitor thread stopped.")
    
    # Start both threads
    read_thread = threading.Thread(target=lidar_read_thread, daemon=True)
    monitor_thread = threading.Thread(target=lidar_monitor_thread, daemon=True)
    read_thread.start()
    monitor_thread.start()
    
    # Get local IP
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = '127.0.0.1'
    finally:
        s.close()
    
    print(f"\n{'='*60}")
    print(f"  Web interface started!")
    print(f"  Access from any device at: http://{local_ip}:{args.port}")
    print(f"  Or locally at: http://localhost:{args.port}")
    print(f"  Max display range: {args.max_distance:.1f} meters")
    print(f"{'='*60}\n")
    print("Press Ctrl+C to stop...")
    
    try:
        app.run(host='0.0.0.0', port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nStopping web server...")
    finally:
        web_view.running = False
        lidar.close()
        print("Web server closed.")


if __name__ == "__main__":
    main()
