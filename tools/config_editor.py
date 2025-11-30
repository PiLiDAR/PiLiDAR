#!/usr/bin/env python3
"""
Web-based Configuration Editor for PiLiDAR

Usage:
    python tools/config_editor.py [--port PORT] [--config PATH]
    
Options:
    --port PORT       Web server port (default: 5001)
    --config PATH     Path to config.json (default: ../config.json)

This script provides a web interface to edit config.json with live validation.
Access from any device on the network at http://<raspberry-pi-ip>:5001
"""

import sys
import os
import argparse
import json
import copy
from typing import Any, Dict, Optional, Tuple
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from flask import Flask, render_template_string, jsonify, request

# HTML template with embedded JavaScript
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>PiLiDAR Config Editor</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #1a1a2e;
            color: #eee;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 30px;
            border-radius: 12px;
            margin-bottom: 30px;
            box-shadow: 0 8px 16px rgba(0,0,0,0.3);
        }
        h1 {
            font-size: 32px;
            font-weight: 300;
            letter-spacing: 1px;
        }
        .subtitle {
            opacity: 0.9;
            margin-top: 8px;
            font-size: 14px;
        }
        .toolbar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: #16213e;
            padding: 15px 20px;
            border-radius: 8px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.2);
        }
        .btn-group {
            display: flex;
            gap: 10px;
        }
        button {
            padding: 10px 20px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 500;
            transition: all 0.3s ease;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.3);
        }
        .btn-primary {
            background: #4CAF50;
            color: white;
        }
        .btn-primary:hover {
            background: #45a049;
        }
        .btn-secondary {
            background: #2196F3;
            color: white;
        }
        .btn-secondary:hover {
            background: #0b7dda;
        }
        .btn-danger {
            background: #f44336;
            color: white;
        }
        .btn-danger:hover {
            background: #da190b;
        }
        .btn-warning {
            background: #ff9800;
            color: white;
        }
        .btn-warning:hover {
            background: #e68900;
        }
        .status {
            padding: 8px 16px;
            border-radius: 6px;
            font-size: 13px;
            font-weight: 500;
        }
        .status.success {
            background: rgba(76, 175, 80, 0.2);
            color: #4CAF50;
            border: 1px solid #4CAF50;
        }
        .status.error {
            background: rgba(244, 67, 54, 0.2);
            color: #f44336;
            border: 1px solid #f44336;
        }
        .status.modified {
            background: rgba(255, 152, 0, 0.2);
            color: #ff9800;
            border: 1px solid #ff9800;
        }
        .editor-container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }
        @media (max-width: 1024px) {
            .editor-container {
                grid-template-columns: 1fr;
            }
        }
        .panel {
            background: #16213e;
            border-radius: 8px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.2);
        }
        .panel h2 {
            font-size: 20px;
            font-weight: 400;
            margin-bottom: 20px;
            color: #667eea;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .section {
            margin-bottom: 25px;
        }
        .section-title {
            font-size: 16px;
            font-weight: 600;
            margin-bottom: 12px;
            color: #a8b2d1;
            display: flex;
            align-items: center;
            cursor: pointer;
            user-select: none;
        }
        .section-title:hover {
            color: #667eea;
        }
        .section-title::before {
            content: '▼';
            margin-right: 8px;
            font-size: 12px;
            transition: transform 0.3s ease;
        }
        .section-title.collapsed::before {
            transform: rotate(-90deg);
        }
        .section-content {
            display: block;
            padding-left: 20px;
        }
        .section-content.collapsed {
            display: none;
        }
        .field {
            margin-bottom: 15px;
        }
        label {
            display: block;
            font-size: 13px;
            margin-bottom: 5px;
            color: #8892b0;
            font-weight: 500;
        }
        input, select, textarea {
            width: 100%;
            padding: 10px;
            border: 1px solid #2a3f5f;
            border-radius: 6px;
            background: #0f1729;
            color: #eee;
            font-size: 14px;
            transition: border-color 0.3s ease;
        }
        input:focus, select:focus, textarea:focus {
            outline: none;
            border-color: #667eea;
        }
        input[type="checkbox"] {
            width: auto;
            margin-right: 8px;
        }
        .checkbox-label {
            display: flex;
            align-items: center;
            cursor: pointer;
        }
        .help-text {
            font-size: 11px;
            color: #6b7c93;
            margin-top: 4px;
            font-style: italic;
        }
        .validation-error {
            font-size: 12px;
            color: #f44336;
            margin-top: 4px;
            display: none;
        }
        .validation-error.show {
            display: block;
        }
        textarea {
            font-family: 'Courier New', monospace;
            resize: vertical;
            min-height: 400px;
        }
        .diff-viewer {
            background: #0f1729;
            border: 1px solid #2a3f5f;
            border-radius: 6px;
            padding: 15px;
            overflow-x: auto;
            max-height: 600px;
            font-family: 'Courier New', monospace;
            font-size: 13px;
            line-height: 1.6;
        }
        .diff-line {
            padding: 2px 0;
        }
        .diff-line.added {
            background: rgba(76, 175, 80, 0.15);
            color: #4CAF50;
        }
        .diff-line.removed {
            background: rgba(244, 67, 54, 0.15);
            color: #f44336;
        }
        .diff-line.unchanged {
            color: #8892b0;
        }
        .modal {
            display: none;
            position: fixed;
            z-index: 1000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.7);
            backdrop-filter: blur(4px);
        }
        .modal-content {
            background: #16213e;
            margin: 5% auto;
            padding: 30px;
            border-radius: 12px;
            max-width: 600px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.4);
        }
        .modal-header {
            font-size: 22px;
            margin-bottom: 20px;
            color: #667eea;
        }
        .modal-body {
            margin-bottom: 20px;
        }
        .modal-footer {
            display: flex;
            justify-content: flex-end;
            gap: 10px;
        }
        .wizard-step {
            display: none;
        }
        .wizard-step.active {
            display: block;
        }
        .wizard-progress {
            display: flex;
            justify-content: space-between;
            margin-bottom: 30px;
        }
        .wizard-progress-item {
            flex: 1;
            text-align: center;
            padding: 10px;
            position: relative;
        }
        .wizard-progress-item::after {
            content: '';
            position: absolute;
            top: 20px;
            left: 50%;
            width: 100%;
            height: 2px;
            background: #2a3f5f;
            z-index: -1;
        }
        .wizard-progress-item:last-child::after {
            display: none;
        }
        .wizard-progress-item.completed {
            color: #4CAF50;
        }
        .wizard-progress-item.active {
            color: #667eea;
            font-weight: 600;
        }
        .offset-diagram {
            width: 100%;
            max-width: 400px;
            margin: 20px auto;
            display: block;
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>🔧 PiLiDAR Configuration Editor</h1>
            <div class="subtitle">Web-based configuration management for Version 0.9.1-beta</div>
        </header>

        <div class="toolbar">
            <div class="btn-group">
                <button class="btn-primary" onclick="saveConfig()">💾 Save Changes</button>
                <button class="btn-secondary" onclick="reloadConfig()">🔄 Reload</button>
                <button class="btn-warning" onclick="openCalibrationWizard()">📐 Calibration Wizard</button>
            </div>
            <div class="status" id="status">Ready</div>
        </div>

        <div class="editor-container">
            <!-- Left Panel: Form Editor -->
            <div class="panel">
                <h2>Configuration Settings</h2>
                <div id="form-editor"></div>
            </div>

            <!-- Right Panel: JSON Preview -->
            <div class="panel">
                <h2>JSON Preview</h2>
                <div class="btn-group" style="margin-bottom: 15px;">
                    <button class="btn-secondary" onclick="formatJson()">✨ Format</button>
                    <button class="btn-secondary" onclick="validateJson()">✓ Validate</button>
                </div>
                <textarea id="json-editor" spellcheck="false"></textarea>
                <div class="validation-error" id="json-error"></div>
            </div>
        </div>
    </div>

    <!-- Calibration Wizard Modal -->
    <div id="calibration-modal" class="modal">
        <div class="modal-content">
            <div class="modal-header">Offset Calibration Wizard</div>
            <div class="modal-body">
                <div class="wizard-progress">
                    <div class="wizard-progress-item active" id="wizard-step-indicator-1">1. Y Offset</div>
                    <div class="wizard-progress-item" id="wizard-step-indicator-2">2. Z Offset</div>
                    <div class="wizard-progress-item" id="wizard-step-indicator-3">3. Verify</div>
                </div>

                <div class="wizard-step active" id="wizard-step-1">
                    <h3>Y Offset Calibration (Horizontal Depth)</h3>
                    <p>Measure the horizontal distance from the stepper motor axis to the LiDAR sensor center.</p>
                    <svg class="offset-diagram" viewBox="0 0 400 200">
                        <rect x="50" y="50" width="100" height="100" fill="#667eea" opacity="0.3"/>
                        <text x="100" y="105" fill="#eee" text-anchor="middle">Motor</text>
                        <rect x="250" y="70" width="60" height="60" fill="#4CAF50" opacity="0.3"/>
                        <text x="280" y="105" fill="#eee" text-anchor="middle">LiDAR</text>
                        <line x1="150" y1="100" x2="250" y2="100" stroke="#f44336" stroke-width="2"/>
                        <text x="200" y="90" fill="#f44336" text-anchor="middle">Y Offset</text>
                    </svg>
                    <div class="field">
                        <label>Y Offset (mm)</label>
                        <input type="number" id="wizard-y-offset" step="0.1" value="-37.5">
                        <div class="help-text">Negative values indicate sensor is in front of motor axis</div>
                    </div>
                </div>

                <div class="wizard-step" id="wizard-step-2">
                    <h3>Z Offset Calibration (Vertical)</h3>
                    <p>Measure the vertical distance from the stepper motor axis to the LiDAR sensor center.</p>
                    <svg class="offset-diagram" viewBox="0 0 400 200">
                        <rect x="150" y="130" width="100" height="50" fill="#667eea" opacity="0.3"/>
                        <text x="200" y="160" fill="#eee" text-anchor="middle">Motor</text>
                        <rect x="170" y="50" width="60" height="40" fill="#4CAF50" opacity="0.3"/>
                        <text x="200" y="75" fill="#eee" text-anchor="middle">LiDAR</text>
                        <line x1="200" y1="90" x2="200" y2="130" stroke="#f44336" stroke-width="2"/>
                        <text x="230" y="110" fill="#f44336" text-anchor="middle">Z Offset</text>
                    </svg>
                    <div class="field">
                        <label>Z Offset (mm)</label>
                        <input type="number" id="wizard-z-offset" step="0.1" value="-41.9">
                        <div class="help-text">Negative values indicate sensor is below motor axis</div>
                    </div>
                </div>

                <div class="wizard-step" id="wizard-step-3">
                    <h3>Verify Calibration</h3>
                    <p>Review your offset values:</p>
                    <div style="background: #0f1729; padding: 15px; border-radius: 6px; margin: 15px 0;">
                        <p><strong>Y Offset:</strong> <span id="verify-y-offset"></span> mm</p>
                        <p><strong>Z Offset:</strong> <span id="verify-z-offset"></span> mm</p>
                    </div>
                    <p style="color: #6b7c93; font-size: 13px;">These values will be applied to the 3D configuration. Click "Apply" to save.</p>
                </div>
            </div>
            <div class="modal-footer">
                <button class="btn-secondary" onclick="wizardPrev()" id="wizard-prev" style="display:none;">← Previous</button>
                <button class="btn-primary" onclick="wizardNext()" id="wizard-next">Next →</button>
                <button class="btn-primary" onclick="wizardApply()" id="wizard-apply" style="display:none;">Apply Changes</button>
                <button class="btn-danger" onclick="closeCalibrationWizard()">Cancel</button>
            </div>
        </div>
    </div>

    <script>
        let config = {};
        let originalConfig = {};
        let currentWizardStep = 1;

        // Load configuration on page load
        window.onload = function() {
            loadConfig();
        };

        async function loadConfig() {
            try {
                const response = await fetch('/api/config');
                const data = await response.json();
                config = data.config;
                originalConfig = JSON.parse(JSON.stringify(config));
                renderFormEditor();
                updateJsonEditor();
                updateStatus('Ready', 'success');
            } catch (error) {
                updateStatus('Error loading config: ' + error.message, 'error');
            }
        }

        function renderFormEditor() {
            const container = document.getElementById('form-editor');
            container.innerHTML = '';

            // Define sections with their fields
            const sections = {
                'General': ['ENABLE_LIDAR', 'ENABLE_CAM', 'ENABLE_IMU', 'ENABLE_PANO', 'ENABLE_3D', 'ENABLE_VERTEXCOLOUR', 'ENABLE_FILTERING'],
                'LiDAR Settings': {
                    parent: 'LIDAR',
                    fields: ['DEVICE', 'LIDAR_OFFSET_ANGLE', 'TARGET_RES', 'TARGET_SPEED']
                },
                'Stepper Motor': {
                    parent: 'STEPPER',
                    fields: ['SCAN_ANGLE', 'GEAR_RATIO', 'MICROSTEPS', 'STEP_DELAY', 'SCAN_DELAY']
                },
                'Camera Settings': {
                    parent: 'CAM',
                    fields: ['preview_dims', 'dims', 'sharpness', 'saturation', 'AEB', 'AEB_STOPS']
                },
                'Panorama': {
                    parent: 'PANO',
                    fields: ['IMGCOUNT', 'PANO_WIDTH']
                },
                '3D Processing': {
                    parent: '3D',
                    fields: ['Y_OFFSET', 'Z_OFFSET', 'NORMAL_RADIUS', 'SCALE', 'EXT', 'ASCII']
                },
                'Filtering': {
                    parent: 'FILTERING',
                    fields: ['FILTER_ON_PI', 'VOXEL_SIZE', 'NB_POINTS', 'RADIUS']
                }
            };

            for (const [sectionName, sectionConfig] of Object.entries(sections)) {
                const section = document.createElement('div');
                section.className = 'section';

                const title = document.createElement('div');
                title.className = 'section-title';
                title.textContent = sectionName;
                title.onclick = () => toggleSection(title);
                section.appendChild(title);

                const content = document.createElement('div');
                content.className = 'section-content';

                let fields, parent;
                if (Array.isArray(sectionConfig)) {
                    fields = sectionConfig;
                    parent = null;
                } else {
                    fields = sectionConfig.fields;
                    parent = sectionConfig.parent;
                }

                fields.forEach(field => {
                    const value = parent ? config[parent]?.[field] : config[field];
                    if (value !== undefined) {
                        content.appendChild(createField(field, value, parent));
                    }
                });

                section.appendChild(content);
                container.appendChild(section);
            }
        }

        function createField(name, value, parent) {
            const field = document.createElement('div');
            field.className = 'field';

            const label = document.createElement('label');
            label.textContent = name.replace(/_/g, ' ');
            field.appendChild(label);

            let input;
            const fieldId = parent ? `${parent}.${name}` : name;

            if (typeof value === 'boolean') {
                const checkboxLabel = document.createElement('label');
                checkboxLabel.className = 'checkbox-label';
                input = document.createElement('input');
                input.type = 'checkbox';
                input.checked = value;
                input.id = fieldId;
                input.onchange = () => updateConfigValue(fieldId, input.checked);
                checkboxLabel.appendChild(input);
                checkboxLabel.appendChild(document.createTextNode(' Enabled'));
                field.appendChild(checkboxLabel);
            } else if (Array.isArray(value)) {
                input = document.createElement('input');
                input.type = 'text';
                input.value = JSON.stringify(value);
                input.id = fieldId;
                input.onchange = () => {
                    try {
                        const parsed = JSON.parse(input.value);
                        updateConfigValue(fieldId, parsed);
                    } catch (e) {
                        alert('Invalid JSON array: ' + e.message);
                    }
                };
                field.appendChild(input);
            } else if (typeof value === 'number') {
                input = document.createElement('input');
                input.type = 'number';
                input.step = value % 1 === 0 ? '1' : '0.01';
                input.value = value;
                input.id = fieldId;
                input.onchange = () => updateConfigValue(fieldId, parseFloat(input.value));
                field.appendChild(input);
            } else {
                input = document.createElement('input');
                input.type = 'text';
                input.value = value;
                input.id = fieldId;
                input.onchange = () => updateConfigValue(fieldId, input.value);
                field.appendChild(input);
            }

            return field;
        }

        function toggleSection(titleElement) {
            titleElement.classList.toggle('collapsed');
            titleElement.nextElementSibling.classList.toggle('collapsed');
        }

        function updateConfigValue(fieldId, value) {
            const parts = fieldId.split('.');
            if (parts.length === 2) {
                if (!config[parts[0]]) config[parts[0]] = {};
                config[parts[0]][parts[1]] = value;
            } else {
                config[parts[0]] = value;
            }
            updateJsonEditor();
            checkModified();
        }

        function updateJsonEditor() {
            document.getElementById('json-editor').value = JSON.stringify(config, null, 4);
        }

        function formatJson() {
            try {
                const text = document.getElementById('json-editor').value;
                const parsed = JSON.parse(text);
                config = parsed;
                updateJsonEditor();
                renderFormEditor();
                updateStatus('JSON formatted', 'success');
            } catch (error) {
                showJsonError('Invalid JSON: ' + error.message);
            }
        }

        function validateJson() {
            try {
                const text = document.getElementById('json-editor').value;
                JSON.parse(text);
                hideJsonError();
                updateStatus('JSON is valid', 'success');
            } catch (error) {
                showJsonError('Invalid JSON: ' + error.message);
                updateStatus('JSON validation failed', 'error');
            }
        }

        function showJsonError(message) {
            const errorDiv = document.getElementById('json-error');
            errorDiv.textContent = message;
            errorDiv.classList.add('show');
        }

        function hideJsonError() {
            const errorDiv = document.getElementById('json-error');
            errorDiv.classList.remove('show');
        }

        function checkModified() {
            const modified = JSON.stringify(config) !== JSON.stringify(originalConfig);
            updateStatus(modified ? 'Modified (unsaved changes)' : 'Ready', modified ? 'modified' : 'success');
        }

        async function saveConfig() {
            try {
                validateJson();
                const response = await fetch('/api/config', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({config: config})
                });
                const result = await response.json();
                if (result.success) {
                    originalConfig = JSON.parse(JSON.stringify(config));
                    updateStatus('Configuration saved successfully', 'success');
                } else {
                    updateStatus('Error saving: ' + result.error, 'error');
                }
            } catch (error) {
                updateStatus('Error saving: ' + error.message, 'error');
            }
        }

        async function reloadConfig() {
            if (JSON.stringify(config) !== JSON.stringify(originalConfig)) {
                if (!confirm('You have unsaved changes. Reload anyway?')) {
                    return;
                }
            }
            await loadConfig();
        }

        function updateStatus(message, type) {
            const status = document.getElementById('status');
            status.textContent = message;
            status.className = 'status ' + type;
        }

        // Calibration Wizard
        function openCalibrationWizard() {
            document.getElementById('calibration-modal').style.display = 'block';
            currentWizardStep = 1;
            updateWizardStep();
            
            // Pre-fill with current values
            document.getElementById('wizard-y-offset').value = config['3D']?.Y_OFFSET || -37.5;
            document.getElementById('wizard-z-offset').value = config['3D']?.Z_OFFSET || -41.9;
        }

        function closeCalibrationWizard() {
            document.getElementById('calibration-modal').style.display = 'none';
        }

        function wizardNext() {
            if (currentWizardStep < 3) {
                currentWizardStep++;
                updateWizardStep();
            }
        }

        function wizardPrev() {
            if (currentWizardStep > 1) {
                currentWizardStep--;
                updateWizardStep();
            }
        }

        function updateWizardStep() {
            // Hide all steps
            document.querySelectorAll('.wizard-step').forEach(step => {
                step.classList.remove('active');
            });
            
            // Show current step
            document.getElementById('wizard-step-' + currentWizardStep).classList.add('active');
            
            // Update progress indicators
            for (let i = 1; i <= 3; i++) {
                const indicator = document.getElementById('wizard-step-indicator-' + i);
                indicator.classList.remove('active', 'completed');
                if (i < currentWizardStep) {
                    indicator.classList.add('completed');
                } else if (i === currentWizardStep) {
                    indicator.classList.add('active');
                }
            }
            
            // Update buttons
            document.getElementById('wizard-prev').style.display = currentWizardStep > 1 ? 'inline-block' : 'none';
            document.getElementById('wizard-next').style.display = currentWizardStep < 3 ? 'inline-block' : 'none';
            document.getElementById('wizard-apply').style.display = currentWizardStep === 3 ? 'inline-block' : 'none';
            
            // Update verification display
            if (currentWizardStep === 3) {
                document.getElementById('verify-y-offset').textContent = document.getElementById('wizard-y-offset').value;
                document.getElementById('verify-z-offset').textContent = document.getElementById('wizard-z-offset').value;
            }
        }

        function wizardApply() {
            const yOffset = parseFloat(document.getElementById('wizard-y-offset').value);
            const zOffset = parseFloat(document.getElementById('wizard-z-offset').value);
            
            if (!config['3D']) config['3D'] = {};
            config['3D'].Y_OFFSET = yOffset;
            config['3D'].Z_OFFSET = zOffset;
            
            updateJsonEditor();
            renderFormEditor();
            checkModified();
            closeCalibrationWizard();
            updateStatus('Offsets updated - remember to save!', 'modified');
        }

        // Handle clicks outside modal
        window.onclick = function(event) {
            const modal = document.getElementById('calibration-modal');
            if (event.target === modal) {
                closeCalibrationWizard();
            }
        };
    </script>
</body>
</html>
'''


class ConfigEditor:
    """Web-based configuration editor."""
    
    def __init__(self, config_path: str):
        """Initialize config editor with path to config.json."""
        self.config_path = os.path.abspath(config_path)
        self.config: Dict[str, Any] = {}
        self.load_config()
    
    def load_config(self) -> None:
        """Load configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            print(f"Warning: Config file not found at {self.config_path}")
            self.config = {}
        except json.JSONDecodeError as e:
            print(f"Error parsing config file: {e}")
            self.config = {}
    
    def save_config(self, new_config: Dict[str, Any]) -> Tuple[bool, Optional[str]]:
        """
        Save configuration to JSON file with validation.
        
        Returns:
            Tuple of (success, error_message)
        """
        try:
            # Validate JSON structure
            json_str = json.dumps(new_config, indent=4)
            
            # Create backup
            backup_path = self.config_path + '.backup'
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    backup_content = f.read()
                with open(backup_path, 'w') as f:
                    f.write(backup_content)
            
            # Write new config
            with open(self.config_path, 'w') as f:
                f.write(json_str)
            
            self.config = new_config
            return True, None
            
        except Exception as e:
            return False, str(e)
    
    def get_config(self) -> Dict[str, Any]:
        """Get current configuration."""
        return copy.deepcopy(self.config)


def create_app(editor: ConfigEditor) -> Flask:
    """Create Flask application."""
    app = Flask(__name__)
    
    @app.route('/')
    def index():
        """Render main page."""
        return render_template_string(HTML_TEMPLATE)
    
    @app.route('/api/config', methods=['GET'])
    def get_config():
        """Get current configuration."""
        return jsonify({
            'success': True,
            'config': editor.get_config()
        })
    
    @app.route('/api/config', methods=['POST'])
    def save_config():
        """Save configuration."""
        try:
            data = request.get_json()
            if not data or 'config' not in data:
                return jsonify({
                    'success': False,
                    'error': 'Missing config data'
                }), 400
            
            success, error = editor.save_config(data['config'])
            if success:
                return jsonify({'success': True})
            else:
                return jsonify({
                    'success': False,
                    'error': error
                }), 500
                
        except Exception as e:
            return jsonify({
                'success': False,
                'error': str(e)
            }), 500
    
    return app


def main():
    """Main function to run config editor server."""
    parser = argparse.ArgumentParser(description='PiLiDAR Configuration Editor')
    parser.add_argument('--port', type=int, default=5001,
                       help='Web server port (default: 5001)')
    parser.add_argument('--config', type=str, default='config.json',
                       help='Path to config.json (default: config.json)')
    args = parser.parse_args()
    
    # Resolve config path
    if not os.path.isabs(args.config):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        args.config = os.path.join(script_dir, '..', args.config)
    
    print("Starting PiLiDAR Configuration Editor...")
    print(f"Config file: {args.config}")
    
    editor = ConfigEditor(args.config)
    app = create_app(editor)
    
    print(f"\n✅ Config Editor ready at http://0.0.0.0:{args.port}")
    print(f"   Local access: http://localhost:{args.port}")
    print(f"   Network access: http://<raspberry-pi-ip>:{args.port}")
    print("\nPress Ctrl+C to stop.")
    
    app.run(host='0.0.0.0', port=args.port, debug=False)


if __name__ == "__main__":
    main()
