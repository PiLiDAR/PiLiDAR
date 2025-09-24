#!/usr/bin/env python3
"""
Fundamentale Koordinaten-Korrektur für PiLIDAR
Korrigiert gekrümmte Wände durch Neukalibrierung der Koordinatentransformation
"""

import numpy as np
import pickle
import os
from scipy.spatial.transform import Rotation as R

def load_lidar_data(lidar_file):
    """Lade LiDAR-Daten aus Pickle-Datei"""
    with open(lidar_file, 'rb') as f:
        return pickle.load(f)

def reconstruct_from_scratch(raw_scan, method='rectangular'):
    """
    Rekonstruiere 3D-Punktwolke komplett neu mit verschiedenen Ansätzen
    
    Methods:
    - 'rectangular': Behandle LiDAR-Koordinaten als rechteckiges Raster
    - 'polar_corrected': Korrigierte polare Transformation
    - 'linear_scan': Behandle jeden Scan als gerade Linie
    """
    cartesian_list = raw_scan.get('cartesian', [])
    z_angles = raw_scan.get('z_angles', [])
    
    print(f'🔧 Rekonstruktion mit Methode: {method}')
    print(f'📊 {len(cartesian_list)} 2D-Scans verfügbar')
    
    all_points = []
    processed_scans = 0
    
    for idx, points2d in enumerate(cartesian_list):
        if not isinstance(points2d, np.ndarray):
            points2d = np.asarray(points2d)
        
        if points2d.size == 0:
            continue
        
        stepper_angle = z_angles[idx] if idx < len(z_angles) else 0.0
        
        # Extrahiere Basis-Koordinaten
        x_vals = points2d[:, 0].astype(np.float64)
        y_vals = points2d[:, 1].astype(np.float64)
        intensity = points2d[:, 2].astype(np.float64) if points2d.shape[1] > 2 else np.zeros_like(x_vals)
        
        if method == 'rectangular':
            # METHODE 1: Behandle X/Y als rechteckiges Koordinatensystem
            # Keine polare Interpretation - direkte kartesische Behandlung
            points3d = np.column_stack((x_vals, y_vals, np.zeros_like(x_vals)))
            
        elif method == 'polar_corrected':
            # METHODE 2: Korrigierte polare Rekonstruktion
            # Zurück zu polaren Koordinaten und neu berechnen
            distances = np.sqrt(x_vals*x_vals + y_vals*y_vals)
            angles = np.arctan2(y_vals, x_vals)
            
            # Korrigiere systematische Verzerrungen
            # LiDAR könnte einen konstanten Winkel-Offset haben
            corrected_angles = angles  # Erstmal ohne Korrektur
            
            # Neue kartesische Koordinaten
            x_corrected = distances * np.cos(corrected_angles)
            y_corrected = distances * np.sin(corrected_angles)
            points3d = np.column_stack((x_corrected, y_corrected, np.zeros_like(x_corrected)))
            
        elif method == 'linear_scan':
            # METHODE 3: Behandle jeden LiDAR-Scan als gerade Linie
            # Das könnte bei rotierenden LiDARs korrekt sein
            
            # Sortiere Punkte nach Winkel für gleichmäßige Verteilung
            angles = np.arctan2(y_vals, x_vals)
            sorted_indices = np.argsort(angles)
            
            x_sorted = x_vals[sorted_indices]
            y_sorted = y_vals[sorted_indices]
            intensity_sorted = intensity[sorted_indices]
            
            # Erstelle gerade Linie durch Punkte
            # Verwende Hauptkomponenten-Analyse (PCA) für beste Gerade
            points_2d = np.column_stack((x_sorted, y_sorted))
            
            if len(points_2d) >= 2:
                # PCA für Hauptachse
                center = np.mean(points_2d, axis=0)
                centered = points_2d - center
                
                if len(centered) > 1:
                    cov_matrix = np.cov(centered.T)
                    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
                    
                    # Hauptrichtung (größter Eigenwert)
                    main_direction = eigenvectors[:, -1]
                    
                    # Projiziere alle Punkte auf diese Hauptachse
                    projections = np.dot(centered, main_direction)
                    
                    # Rekonstruiere Punkte auf gerader Linie
                    straight_points = center + projections[:, np.newaxis] * main_direction
                    points3d = np.column_stack((straight_points, np.zeros(len(straight_points))))
                    intensity = intensity_sorted  # Behalte Intensitäten
                else:
                    points3d = np.column_stack((x_sorted, y_sorted, np.zeros_like(x_sorted)))
            else:
                points3d = np.column_stack((x_vals, y_vals, np.zeros_like(x_vals)))
                
        else:
            raise ValueError(f'Unbekannte Methode: {method}')
        
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
        print('❌ Keine Punkte verarbeitet!')
        return None
    
    # Zusammenfügen aller Punkte
    merged_points = np.concatenate(all_points, axis=0)
    
    # Entferne NaN-Werte
    valid_mask = ~np.isnan(merged_points).any(axis=1)
    merged_points = merged_points[valid_mask]
    
    print(f'✅ Rekonstruierte Punktwolke: {len(merged_points):,} Punkte')
    
    return merged_points

def analyze_wall_straightness(points, num_slices=10):
    """
    Analysiere die Geradheit von Wänden in verschiedenen Schnittebenen
    """
    print('📏 DETAILLIERTE GERADHEIT-ANALYSE:')
    
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    z_coords = points[:, 2]
    
    # Analysiere horizontale Schnitte (konstante Z-Werte)
    z_min, z_max = z_coords.min(), z_coords.max()
    z_range = z_max - z_min
    
    if z_range > 10:  # Nur wenn 3D-Struktur vorhanden
        straightness_scores = []
        
        for i in range(num_slices):
            z_slice = z_min + (i + 0.5) * z_range / num_slices
            
            # Wähle Punkte in diesem Z-Bereich
            tolerance = z_range / (2 * num_slices)
            mask = np.abs(z_coords - z_slice) < tolerance
            slice_points = points[mask]
            
            if len(slice_points) > 10:  # Genug Punkte für Analyse
                # Analysiere Geradheit in XY-Ebene
                xy_points = slice_points[:, :2]
                
                # Finde dominante Richtungen (Wände)
                # Einfache Methode: Analysiere Verteilung in radialen Richtungen
                angles = np.arctan2(xy_points[:, 1], xy_points[:, 0])
                
                # Teile in Winkelbereiche (z.B. 4 Hauptrichtungen für rechteckigen Raum)
                angle_bins = 8
                angle_step = 2 * np.pi / angle_bins
                
                straightness_in_directions = []
                
                for j in range(angle_bins):
                    angle_center = j * angle_step
                    angle_tolerance = angle_step / 2
                    
                    # Normalisiere Winkel
                    angle_diff = angles - angle_center
                    angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                    
                    direction_mask = np.abs(angle_diff) < angle_tolerance
                    direction_points = xy_points[direction_mask]
                    
                    if len(direction_points) > 5:
                        # Berechne Geradheit für diese Richtung
                        distances = np.sqrt(np.sum(direction_points**2, axis=1))
                        
                        if len(np.unique(distances)) > 1:
                            # Variationskoeffizient der Abstände
                            cv = np.std(distances) / np.mean(distances)
                            straightness = 1.0 / (1.0 + cv)  # Höhere Werte = gerader
                            straightness_in_directions.append(straightness)
                
                if straightness_in_directions:
                    avg_straightness = np.mean(straightness_in_directions)
                    straightness_scores.append(avg_straightness)
                    print(f'  Z-Schnitt {z_slice:.0f}mm: Geradheit = {avg_straightness:.3f}')
        
        if straightness_scores:
            overall_straightness = np.mean(straightness_scores)
            print(f'\\n📊 Gesamte Wand-Geradheit: {overall_straightness:.3f} (1.0 = perfekt gerade)')
            return overall_straightness
    
    print('⚠️  Nicht genug 3D-Struktur für Wand-Analyse')
    return 0.0

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
    print('=== FUNDAMENTALE KOORDINATEN-KORREKTUR ===')
    
    scan_dir = 'scans/250924-1047'
    lidar_file = os.path.join(scan_dir, '250924-1047_lidar.pkl')
    
    # Lade LiDAR-Daten
    print(f'📁 Lade LiDAR-Daten: {lidar_file}')
    raw_scan = load_lidar_data(lidar_file)
    
    # Teste verschiedene Rekonstruktionsmethoden
    methods = ['rectangular', 'polar_corrected', 'linear_scan']
    
    best_method = None
    best_straightness = 0.0
    
    for method in methods:
        print(f'\\n🔧 TESTE METHODE: {method.upper()}')
        
        try:
            points = reconstruct_from_scratch(raw_scan, method=method)
            
            if points is not None:
                # Basis-Analyse
                x_coords = points[:, 0]
                y_coords = points[:, 1]
                z_coords = points[:, 2]
                
                print(f'📊 Punktwolken-Statistik:')
                print(f'  Anzahl Punkte: {len(points):,}')
                print(f'  X-Bereich: {x_coords.min():.0f} bis {x_coords.max():.0f}mm')
                print(f'  Y-Bereich: {y_coords.min():.0f} bis {y_coords.max():.0f}mm')
                print(f'  Z-Bereich: {z_coords.min():.0f} bis {z_coords.max():.0f}mm')
                
                # Geradheit-Analyse
                straightness = analyze_wall_straightness(points)
                
                if straightness > best_straightness:
                    best_straightness = straightness
                    best_method = method
                
                # Speichere Ergebnis
                ply_filename = f'250924-1047_METHOD_{method}_intensity.ply'
                ply_filepath = os.path.join(scan_dir, ply_filename)
                save_ply_simple(points, ply_filepath)
            
        except Exception as e:
            print(f'❌ Fehler bei Methode {method}: {e}')
    
    print(f'\\n🏆 BESTE METHODE: {best_method} (Geradheit: {best_straightness:.3f})')
    print(f'\\n🎯 EMPFEHLUNG:')
    print(f'Vergleiche die PLY-Dateien in einem 3D-Viewer:')
    for method in methods:
        print(f'  - {scan_dir}/250924-1047_METHOD_{method}_intensity.ply')
    print(f'Die Datei mit den geradesten Wänden ist die korrekte Lösung!')