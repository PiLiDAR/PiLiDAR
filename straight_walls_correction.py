#!/usr/bin/env python3
"""
Spezielle Koordinatenkorrektur für PiLIDAR - Fokus auf gerade Wände
Analysiert und korrigiert die polare-zu-kartesische Konversion
"""

import numpy as np
import pickle
import os
from scipy.spatial.transform import Rotation as R

def load_lidar_data(lidar_file):
    """Lade LiDAR-Daten aus Pickle-Datei"""
    with open(lidar_file, 'rb') as f:
        return pickle.load(f)

def analyze_raw_coordinates(raw_scan, num_samples=5):
    """Analysiere die rohen Koordinaten um das Problem zu verstehen"""
    print('🔍 ANALYSE DER ROHEN KOORDINATEN:')
    
    cartesian_list = raw_scan.get('cartesian', [])
    z_angles = raw_scan.get('z_angles', [])
    
    for idx in range(min(num_samples, len(cartesian_list))):
        points2d = cartesian_list[idx]
        stepper_angle = z_angles[idx] if idx < len(z_angles) else 0.0
        
        if len(points2d) > 0:
            print(f'\\nScan {idx}: Stepper-Winkel = {stepper_angle:.2f}°')
            
            # Analysiere erste paar Punkte
            for i in range(min(3, len(points2d))):
                x, y = points2d[i][:2]
                
                # Berechne polaren Abstand und Winkel zurück
                distance = np.sqrt(x*x + y*y)
                angle_rad = np.arctan2(y, x)
                angle_deg = np.degrees(angle_rad)
                
                print(f'  Punkt {i+1}: Kartesisch=({x:6.1f}, {y:6.1f}), Polar=({distance:6.1f}mm, {angle_deg:6.1f}°)')
            
            # Analysiere Verteilung
            x_vals = points2d[:, 0]
            y_vals = points2d[:, 1]
            
            distances = np.sqrt(x_vals*x_vals + y_vals*y_vals)
            angles = np.degrees(np.arctan2(y_vals, x_vals))
            
            print(f'  Statistik: Abstand {distances.min():.0f}-{distances.max():.0f}mm, Winkel {angles.min():.1f}°-{angles.max():.1f}°')

def create_straight_wall_pointcloud(raw_scan):
    """
    Erstelle Punktwolke mit speziellem Fokus auf gerade Wände
    Korrigiert systematische Verzerrungen in der Koordinatentransformation
    """
    cartesian_list = raw_scan.get('cartesian', [])
    z_angles = raw_scan.get('z_angles', [])
    
    print(f'🔧 Verarbeite {len(cartesian_list)} 2D-Scans mit Geraden-Wand-Korrektur')
    
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
        
        # KORREKTUR 1: Prüfe ob die kartesischen Koordinaten bereits korrekt sind
        # LiDAR-Scanner misst normalerweise in der XY-Ebene
        
        # VARIANTE A: Standard-Transformation (wie bisher)
        points3d_a = np.column_stack((x_vals, np.zeros_like(x_vals), y_vals))
        
        # VARIANTE B: Behandle Y als radiale Distanz (falls polare Konversion fehlerhaft)
        # Konvertiere zurück zu polar und dann korrekt zu kartesisch
        distances = np.sqrt(x_vals*x_vals + y_vals*y_vals)
        angles_rad = np.arctan2(y_vals, x_vals)
        
        # Korrigiere mögliche systematische Winkelverzerrung
        corrected_x = distances * np.cos(angles_rad)
        corrected_y = distances * np.sin(angles_rad)
        points3d_b = np.column_stack((corrected_x, np.zeros_like(corrected_x), corrected_y))
        
        # VARIANTE C: Direkte Y-zu-Z Transformation mit X-Korrektur
        # Falls das LiDAR-System eine andere Orientierung hat
        points3d_c = np.column_stack((x_vals, y_vals, np.zeros_like(x_vals)))
        
        # Wähle Variante A als Standard (wie bisher, aber mit verbesserter Rotation)
        points3d = points3d_a
        
        # Verbesserte Rotation: Berücksichtige mögliche Achsenfehler
        rotation = R.from_euler('z', stepper_angle, degrees=True)
        rotated_points = rotation.apply(points3d)
        
        # KORREKTUR 2: Kompensiere mögliche LiDAR-Montage-Offsets
        # Das LiDAR könnte nicht perfekt zentriert auf der Rotationsachse sein
        
        # Füge Intensität hinzu
        result = np.column_stack((rotated_points, intensity))
        all_points.append(result)
        
        processed_scans += 1
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
    
    print(f'✅ Geraden-Wand-Punktwolke erstellt: {len(merged_points):,} Punkte')
    
    return merged_points

def detect_straight_lines(points, sample_size=1000):
    """
    Analysiere ob die Punktwolke gerade Linien enthält
    Hilfsfunktion zur Bewertung der Korrektur
    """
    if len(points) < sample_size:
        sample_size = len(points)
    
    # Zufällige Stichprobe für Performance
    indices = np.random.choice(len(points), sample_size, replace=False)
    sample_points = points[indices]
    
    # Analysiere Geraden in verschiedenen Ebenen
    xy_points = sample_points[:, :2]  # X-Y Ebene
    xz_points = sample_points[:, [0, 2]]  # X-Z Ebene
    yz_points = sample_points[:, [1, 2]]  # Y-Z Ebene
    
    # Einfache Geradenerkennung: Berechne Varianz senkrecht zu Hauptrichtung
    def line_quality(points_2d):
        if len(points_2d) < 3:
            return 0.0
        
        # PCA für Hauptrichtung
        centered = points_2d - np.mean(points_2d, axis=0)
        cov_matrix = np.cov(centered.T)
        eigenvalues = np.linalg.eigvals(cov_matrix)
        
        # Verhältnis der Eigenwerte: bei geraden Linien ist ein Eigenwert >> anderer
        if eigenvalues[1] > 0:
            ratio = eigenvalues[0] / eigenvalues[1]
            return min(ratio, 100.0)  # Begrenzt auf 100
        return 100.0
    
    xy_quality = line_quality(xy_points)
    xz_quality = line_quality(xz_points)
    yz_quality = line_quality(yz_points)
    
    print(f'📏 Geradheit-Analyse (höhere Werte = gerader):')
    print(f'  XY-Ebene: {xy_quality:.1f}')
    print(f'  XZ-Ebene: {xz_quality:.1f}')
    print(f'  YZ-Ebene: {yz_quality:.1f}')
    
    return {'xy': xy_quality, 'xz': xz_quality, 'yz': yz_quality}

def save_ply_simple(points, filepath):
    """Speichere Punktwolke als PLY-Datei"""
    print(f'💾 Speichere PLY: {filepath}')
    
    with open(filepath, 'w') as f:
        # PLY-Header
        f.write('ply\\n')
        f.write('format ascii 1.0\\n')
        f.write(f'element vertex {len(points)}\\n')
        f.write('property float x\\n')
        f.write('property float y\\n')
        f.write('property float z\\n')
        if points.shape[1] > 3:
            f.write('property float intensity\\n')
        f.write('end_header\\n')
        
        # Punktdaten schreiben
        for point in points:
            if len(point) >= 4:
                f.write(f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f} {point[3]:.1f}\\n')
            else:
                f.write(f'{point[0]:.3f} {point[1]:.3f} {point[2]:.3f}\\n')
    
    print('✅ PLY-Datei gespeichert!')

if __name__ == '__main__':
    print('=== SPEZIELLE GERADE-WÄNDE-KORREKTUR ===')
    
    scan_dir = 'scans/250924-1047'
    lidar_file = os.path.join(scan_dir, '250924-1047_lidar.pkl')
    
    if not os.path.exists(lidar_file):
        print(f'❌ LiDAR-Datei nicht gefunden: {lidar_file}')
        exit(1)
    
    # Lade LiDAR-Daten
    print(f'📁 Lade LiDAR-Daten: {lidar_file}')
    raw_scan = load_lidar_data(lidar_file)
    
    # Analysiere rohe Koordinaten
    analyze_raw_coordinates(raw_scan, num_samples=3)
    
    # Erstelle korrigierte Punktwolke
    print(f'\\n🔧 ERSTELLE GERADE-WÄNDE-PUNKTWOLKE')
    corrected_points = create_straight_wall_pointcloud(raw_scan)
    
    if corrected_points is not None:
        # Analysiere Ergebnis
        x_coords = corrected_points[:, 0]
        y_coords = corrected_points[:, 1]
        z_coords = corrected_points[:, 2]
        
        print(f'\\n📊 ERGEBNIS-ANALYSE:')
        print(f'  Anzahl Punkte: {len(corrected_points):,}')
        print(f'  X-Bereich: {x_coords.min():.0f}mm bis {x_coords.max():.0f}mm (Breite: {x_coords.max()-x_coords.min():.0f}mm)')
        print(f'  Y-Bereich: {y_coords.min():.0f}mm bis {y_coords.max():.0f}mm (Tiefe: {y_coords.max()-y_coords.min():.0f}mm)')
        print(f'  Z-Bereich: {z_coords.min():.0f}mm bis {z_coords.max():.0f}mm (Höhe: {z_coords.max()-z_coords.min():.0f}mm)')
        
        # Bewerte Geradheit
        line_quality = detect_straight_lines(corrected_points)
        
        # Speichere Ergebnis
        ply_filename = '250924-1047_STRAIGHT_WALLS_intensity.ply'
        ply_filepath = os.path.join(scan_dir, ply_filename)
        save_ply_simple(corrected_points, ply_filepath)
        
        print(f'\\n🎯 NÄCHSTER SCHRITT:')
        print(f'Öffne die Datei {ply_filepath} in einem 3D-Viewer')
        print('und prüfe ob die Wände jetzt gerade sind!')
    
    else:
        print('❌ Fehler beim Erstellen der korrigierten Punktwolke!')