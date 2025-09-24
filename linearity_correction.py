#!/usr/bin/env python3
"""
Lineare Kalibrationskorrektur für PiLIDAR
Korrigiert systematische Verzerrungen die gerade Wände krümmen
"""

import numpy as np
import pickle
import os
from scipy.spatial.transform import Rotation as R

def load_lidar_data(lidar_file):
    """Lade LiDAR-Daten aus Pickle-Datei"""
    with open(lidar_file, 'rb') as f:
        return pickle.load(f)

def apply_linearity_correction(x_vals, y_vals, correction_type='radial'):
    """
    Wende Linearitätskorrekturen an um gekrümmte Wände zu begradigen
    
    Correction types:
    - 'radial': Korrigiere radiale Verzerrung (häufig bei optischen Systemen)
    - 'angular': Korrigiere Winkelverzerrung (bei rotierenden Scannern)
    - 'barrel': Korrigiere Barrel-Verzerrung (Tonnenverzerrung)
    - 'distance': Korrigiere distanzabhängige Verzerrung
    """
    x_corrected = x_vals.copy()
    y_corrected = y_vals.copy()
    
    if correction_type == 'radial':
        # Radiale Verzerrungskorrektur: r_corrected = r * (1 + k1*r^2 + k2*r^4)
        distances = np.sqrt(x_vals*x_vals + y_vals*y_vals)
        
        # Experimentelle Koeffizienten für PiLIDAR
        k1 = -0.0001  # Negative Werte korrigieren Barrel-Verzerrung
        k2 = 0.0000001
        
        # Normalisierung (Referenzdistanz = 1000mm)
        r_norm = distances / 1000.0
        correction_factor = 1.0 + k1 * r_norm**2 + k2 * r_norm**4
        
        corrected_distances = distances * correction_factor
        
        # Zurück zu kartesischen Koordinaten
        angles = np.arctan2(y_vals, x_vals)
        x_corrected = corrected_distances * np.cos(angles)
        y_corrected = corrected_distances * np.sin(angles)
        
    elif correction_type == 'angular':
        # Winkelverzerrungskorrektur
        angles = np.arctan2(y_vals, x_vals)
        distances = np.sqrt(x_vals*x_vals + y_vals*y_vals)
        
        # Experimentelle Winkelkorrektur
        angle_correction = 0.02 * np.sin(4 * angles)  # Korrigiere 4-fache Symmetrie
        corrected_angles = angles + angle_correction
        
        x_corrected = distances * np.cos(corrected_angles)
        y_corrected = distances * np.sin(corrected_angles)
        
    elif correction_type == 'barrel':
        # Barrel-Verzerrungskorrektur (Tonnenverzerrung)
        center_x, center_y = 0.0, 0.0  # Annahme: Zentrum bei (0,0)
        
        dx = x_vals - center_x
        dy = y_vals - center_y
        r = np.sqrt(dx*dx + dy*dy)
        
        # Barrel-Korrektur-Parameter
        k1 = -0.0001
        r_norm = r / 1000.0
        correction = 1.0 + k1 * r_norm**2
        
        x_corrected = center_x + dx * correction
        y_corrected = center_y + dy * correction
        
    elif correction_type == 'distance':
        # Distanzabhängige Korrektur
        distances = np.sqrt(x_vals*x_vals + y_vals*y_vals)
        
        # Lineare Distanzkorrektur
        # Nahbereich wird gedehnt, Fernbereich komprimiert
        distance_factor = 1.0 + 0.0001 * (distances - 1000.0)
        
        x_corrected = x_vals * distance_factor
        y_corrected = y_vals * distance_factor
    
    return x_corrected, y_corrected

def create_linearity_corrected_pointcloud(raw_scan, correction_type='radial'):
    """
    Erstelle Punktwolke mit Linearitätskorrektur
    """
    cartesian_list = raw_scan.get('cartesian', [])
    z_angles = raw_scan.get('z_angles', [])
    
    print(f'🔧 Verarbeite mit Linearitätskorrektur: {correction_type}')
    
    all_points = []
    processed_scans = 0
    
    for idx, points2d in enumerate(cartesian_list):
        if not isinstance(points2d, np.ndarray):
            points2d = np.asarray(points2d)
        
        if points2d.size == 0:
            continue
            
        stepper_angle = z_angles[idx] if idx < len(z_angles) else 0.0
        
        # Extrahiere 2D-Koordinaten
        x_vals = points2d[:, 0].astype(np.float64)
        y_vals = points2d[:, 1].astype(np.float64)
        intensity = points2d[:, 2].astype(np.float64) if points2d.shape[1] > 2 else np.zeros_like(x_vals)
        
        # Wende Linearitätskorrektur an
        x_corrected, y_corrected = apply_linearity_correction(x_vals, y_vals, correction_type)
        
        # 3D-Transformation: X bleibt X, Y wird zu Z-Höhe
        points3d = np.column_stack((x_corrected, np.zeros_like(x_corrected), y_corrected))
        
        # Rotiere um Z-Achse (Stepper-Rotation)
        if stepper_angle != 0:
            rotation = R.from_euler('z', stepper_angle, degrees=True)
            rotated_points = rotation.apply(points3d)
        else:
            rotated_points = points3d
        
        # Füge Intensität hinzu
        result = np.column_stack((rotated_points, intensity))
        all_points.append(result)
        
        processed_scans += 1
        if processed_scans % 5000 == 0:
            print(f'  Verarbeitet: {processed_scans} von {len(cartesian_list)} Scans')
    
    if not all_points:
        return None
    
    # Zusammenfügen
    merged_points = np.concatenate(all_points, axis=0)
    valid_mask = ~np.isnan(merged_points).any(axis=1)
    merged_points = merged_points[valid_mask]
    
    print(f'✅ Linearitäts-korrigierte Punktwolke: {len(merged_points):,} Punkte')
    
    return merged_points

def save_ply_simple(points, filepath):
    """Speichere Punktwolke als PLY-Datei"""
    print(f'💾 Speichere PLY: {filepath}')
    
    with open(filepath, 'w') as f:
        f.write('ply\\n')
        f.write('format ascii 1.0\\n')
        f.write(f'element vertex {len(points)}\\n')
        f.write('property float x\\n')
        f.write('property float y\\n')
        f.write('property float z\\n')
        if points.shape[1] > 3:
            f.write('property float intensity\\n')
        f.write('end_header\\n')
        
        for point in points:
            if len(point) >= 4:
                f.write(f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f} {point[3]:.1f}\\n')
            else:
                f.write(f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}\\n')
    
    print('✅ PLY-Datei gespeichert!')

if __name__ == '__main__':
    print('=== LINEARE KALIBRATIONS-KORREKTUR ===')
    
    scan_dir = 'scans/250924-1047'
    lidar_file = os.path.join(scan_dir, '250924-1047_lidar.pkl')
    
    # Lade LiDAR-Daten
    print(f'📁 Lade LiDAR-Daten: {lidar_file}')
    raw_scan = load_lidar_data(lidar_file)
    
    # Teste verschiedene Linearitätskorrekturen
    corrections = ['radial', 'angular', 'barrel', 'distance']
    
    for correction in corrections:
        print(f'\\n🔧 TESTE KORREKTUR: {correction.upper()}')
        
        try:
            points = create_linearity_corrected_pointcloud(raw_scan, correction_type=correction)
            
            if points is not None:
                x_coords = points[:, 0]
                y_coords = points[:, 1]
                z_coords = points[:, 2]
                
                print(f'📊 Punktwolken-Statistik:')
                print(f'  Anzahl Punkte: {len(points):,}')
                print(f'  X-Bereich: {x_coords.min():.0f} bis {x_coords.max():.0f}mm')
                print(f'  Y-Bereich: {y_coords.min():.0f} bis {y_coords.max():.0f}mm')  
                print(f'  Z-Bereich: {z_coords.min():.0f} bis {z_coords.max():.0f}mm')
                
                height_range = z_coords.max() - z_coords.min()
                print(f'  Höhendifferenz: {height_range:.0f}mm')
                
                # Speichere Ergebnis
                ply_filename = f'250924-1047_LINEARITY_{correction}_intensity.ply'
                ply_filepath = os.path.join(scan_dir, ply_filename)
                save_ply_simple(points, ply_filepath)
                
        except Exception as e:
            print(f'❌ Fehler bei Korrektur {correction}: {e}')
    
    print(f'\\n🎯 NÄCHSTER SCHRITT:')
    print(f'Vergleiche die PLY-Dateien in einem 3D-Viewer:')
    for correction in corrections:
        print(f'  - {scan_dir}/250924-1047_LINEARITY_{correction}_intensity.ply')
    print(f'\\nSuche nach der Datei mit den geradesten Wänden!')
    print(f'Falls alle noch gekrümmt sind, könnte das Problem in der')
    print(f'Hardware-Kalibrierung oder Montage des LiDAR-Systems liegen.')