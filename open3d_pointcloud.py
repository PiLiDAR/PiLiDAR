#!/usr/bin/env python3
"""
Open3D-basierte Punktwolken-Verarbeitung mit korrigierter Koordinatentransformation
für PiLIDAR - Alternative zum gekrümmte-Wände-Problem
"""

import numpy as np
import pickle
import os
from scipy.spatial.transform import Rotation as R

def load_lidar_data(lidar_file):
    """Lade LiDAR-Daten aus Pickle-Datei"""
    with open(lidar_file, 'rb') as f:
        return pickle.load(f)

def correct_stepper_angles(raw_scan, total_rotation=360.0):
    """
    Korrigiere die Stepper-Winkel falls sie alle 0.0° sind
    Verteile die Winkel gleichmäßig über die Gesamtrotation
    """
    z_angles = raw_scan.get('z_angles', [])
    cartesian_list = raw_scan.get('cartesian', [])
    
    if not z_angles or all(angle == 0.0 for angle in z_angles):
        print('⚠️  Z-Winkel sind alle 0.0° - korrigiere automatisch')
        num_scans = len(cartesian_list)
        if num_scans > 0:
            # Verteile Winkel gleichmäßig über 360°
            corrected_angles = np.linspace(0, total_rotation, num_scans, endpoint=False)
            print(f'✅ Korrigierte Winkel: 0° bis {total_rotation}° in {num_scans} Schritten')
            return corrected_angles
    else:
        print(f'✅ Z-Winkel korrekt: {len(z_angles)} Winkel vorhanden')
        return np.array(z_angles)
    
    return np.zeros(len(cartesian_list))

def transform_2d_to_3d_corrected(points2d, stepper_angle_deg, method='corrected'):
    """
    Transformiere 2D-LiDAR-Punkte zu 3D mit verschiedenen Methoden
    
    Args:
        points2d: NumPy array mit 2D-Punkten [x, y, intensity]
        stepper_angle_deg: Stepper-Winkel in Grad
        method: 'original', 'corrected', 'cylindrical'
    """
    if len(points2d) == 0:
        return np.zeros((0, 4))
    
    # Extrahiere Koordinaten
    x_vals = points2d[:, 0].astype(np.float64)
    y_vals = points2d[:, 1].astype(np.float64) 
    intensity = points2d[:, 2].astype(np.float64) if points2d.shape[1] > 2 else np.zeros_like(x_vals)
    
    if method == 'original':
        # Ursprüngliche Methode: X bleibt X, Y wird zu Z, Z=0 wird zu Y
        points3d = np.column_stack((x_vals, np.zeros_like(x_vals), y_vals))
        
    elif method == 'corrected':
        # Korrigierte Methode: Behandle LiDAR-Ebene als XY, rotiere um Z
        points3d = np.column_stack((x_vals, y_vals, np.zeros_like(x_vals)))
        
    elif method == 'cylindrical':
        # Zylindrische Methode: Y-Werte werden zu Höhen
        points3d = np.column_stack((x_vals, np.zeros_like(x_vals), y_vals))
        
    else:
        raise ValueError(f"Unbekannte Methode: {method}")
    
    # Rotiere um die Z-Achse (Stepper dreht horizontal)
    rotation = R.from_euler('z', stepper_angle_deg, degrees=True)
    rotated_points = rotation.apply(points3d)
    
    # Füge Intensität hinzu
    result = np.column_stack((rotated_points, intensity))
    
    return result

def create_pointcloud_open3d(raw_scan, method='corrected'):
    """
    Erstelle 3D-Punktwolke mit Open3D-kompatibler Transformation
    """
    try:
        import open3d as o3d
        print('✅ Open3D verfügbar')
        use_open3d = True
    except ImportError:
        print('⚠️  Open3D nicht verfügbar - verwende NumPy-Alternative')
        use_open3d = False
    
    cartesian_list = raw_scan.get('cartesian', [])
    
    # Korrigiere Stepper-Winkel
    corrected_angles = correct_stepper_angles(raw_scan)
    
    print(f'🔄 Verarbeite {len(cartesian_list)} 2D-Scans mit Methode: {method}')
    
    all_points = []
    processed_scans = 0
    
    for idx, points2d in enumerate(cartesian_list):
        if not isinstance(points2d, np.ndarray):
            points2d = np.asarray(points2d)
        
        if points2d.size == 0:
            continue
            
        stepper_angle = corrected_angles[idx] if idx < len(corrected_angles) else 0.0
        
        # Transformiere mit gewählter Methode
        points3d = transform_2d_to_3d_corrected(points2d, stepper_angle, method)
        
        if len(points3d) > 0:
            all_points.append(points3d)
            processed_scans += 1
            
        # Fortschritt anzeigen
        if processed_scans % 5000 == 0:
            print(f'  Verarbeitet: {processed_scans} von {len(cartesian_list)} Scans')
    
    if not all_points:
        print('❌ Keine Punkte verarbeitet!')
        return None
    
    # Zusammenfügen aller Punkte
    merged_points = np.concatenate(all_points, axis=0)
    
    # Entferne NaN-Werte
    valid_mask = ~np.isnan(merged_points).any(axis=1)
    merged_points = merged_points[valid_mask]
    
    print(f'✅ 3D-Punktwolke erstellt: {len(merged_points):,} Punkte')
    
    if use_open3d:
        # Erstelle Open3D-Punktwolke
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(merged_points[:, :3])
        
        # Füge Farben basierend auf Intensität hinzu
        if merged_points.shape[1] > 3:
            intensities = merged_points[:, 3]
            # Normalisiere Intensitäten zu Farben (Graustufen)
            colors = np.column_stack([intensities/255.0] * 3)
            pcd.colors = o3d.utility.Vector3dVector(colors)
        
        return pcd, merged_points
    else:
        return None, merged_points

def analyze_pointcloud(points, method_name):
    """Analysiere die 3D-Punktwolke"""
    if points is None or len(points) == 0:
        print(f'❌ {method_name}: Keine Punkte verfügbar!')
        return
    
    x_coords = points[:, 0]
    y_coords = points[:, 1] 
    z_coords = points[:, 2]
    
    print(f'\\n📊 {method_name} - Punktwolken-Analyse:')
    print(f'  Anzahl Punkte: {len(points):,}')
    print(f'  X-Bereich: {x_coords.min():.0f}mm bis {x_coords.max():.0f}mm (Breite: {x_coords.max()-x_coords.min():.0f}mm)')
    print(f'  Y-Bereich: {y_coords.min():.0f}mm bis {y_coords.max():.0f}mm (Tiefe: {y_coords.max()-y_coords.min():.0f}mm)')
    print(f'  Z-Bereich: {z_coords.min():.0f}mm bis {z_coords.max():.0f}mm (Höhe: {z_coords.max()-z_coords.min():.0f}mm)')
    
    height_range = z_coords.max() - z_coords.min()
    if height_range > 10:
        print(f'  ✅ 3D-Struktur: {height_range:.0f}mm Höhendifferenz')
    else:
        print(f'  ⚠️  Flache Struktur: {height_range:.1f}mm Höhendifferenz')

def save_ply_simple(points, filepath):
    """Speichere Punktwolke als PLY-Datei"""
    print(f'💾 Speichere PLY: {filepath}')
    
    with open(filepath, 'w') as f:
        # PLY-Header
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {len(points)}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        if points.shape[1] > 3:
            f.write('property float intensity\n')
        f.write('end_header\n')
        
        # Punktdaten schreiben
        for point in points:
            if len(point) >= 4:
                f.write(f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f} {point[3]:.1f}\n')
            else:
                f.write(f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}\n')
    
    print('✅ PLY-Datei gespeichert!')

if __name__ == '__main__':
    # Hauptprogramm
    print('=== OPEN3D-BASIERTE PUNKTWOLKEN-KORREKTUR ===')
    
    scan_dir = 'scans/250924-1047'
    lidar_file = os.path.join(scan_dir, '250924-1047_lidar.pkl')
    
    if not os.path.exists(lidar_file):
        print(f'❌ LiDAR-Datei nicht gefunden: {lidar_file}')
        exit(1)
    
    # Lade LiDAR-Daten
    print(f'📁 Lade LiDAR-Daten: {lidar_file}')
    raw_scan = load_lidar_data(lidar_file)
    
    # Teste verschiedene Transformationsmethoden
    methods = ['original', 'corrected', 'cylindrical']
    
    for method in methods:
        print(f'\\n🔧 TESTE METHODE: {method.upper()}')
        
        try:
            pcd_o3d, points_np = create_pointcloud_open3d(raw_scan, method=method)
            analyze_pointcloud(points_np, method)
            
            # Speichere Ergebnis
            ply_filename = f'250924-1047_open3d_{method}_intensity.ply'
            ply_filepath = os.path.join(scan_dir, ply_filename)
            save_ply_simple(points_np, ply_filepath)
            
        except Exception as e:
            print(f'❌ Fehler bei Methode {method}: {e}')
    
    print('\\n🎯 EMPFEHLUNG:')
    print('Vergleiche die drei PLY-Dateien in einem 3D-Viewer:')
    print(f'  - {scan_dir}/250924-1047_open3d_original_intensity.ply')
    print(f'  - {scan_dir}/250924-1047_open3d_corrected_intensity.ply') 
    print(f'  - {scan_dir}/250924-1047_open3d_cylindrical_intensity.ply')
    print('Die Methode mit geraden Wänden ist die korrekte!')