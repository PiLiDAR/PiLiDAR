#!/usr/bin/env python3
"""
Demonstriert den Unterschied zwischen Translation vor und nach Rotation
Zeigt warum Y_OFFSET vor der Rotation angewendet werden muss für gerade Wände
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def demonstrate_rotation_order():
    """Demonstriert den Effekt der Rotationsreihenfolge auf Y_OFFSET"""
    
    print("=== DEMONSTRATION: Y_OFFSET ROTATIONSREIHENFOLGE ===")
    
    # Simuliere LiDAR-Punkte einer geraden Wand
    wall_distance = 1000  # 1000mm Entfernung
    lidar_angles = np.linspace(-30, 30, 61)  # -30° bis +30° LiDAR-Scan
    
    # Kartesische Koordinaten der Wand (gerade Linie)
    x_vals = wall_distance * np.cos(np.radians(lidar_angles))
    y_vals = wall_distance * np.sin(np.radians(lidar_angles))
    z_vals = np.zeros_like(x_vals)  # LiDAR-Ebene
    
    # 3D-Punkte zusammenfügen
    points3d = np.column_stack((x_vals, z_vals, y_vals))  # (X, Z, Y) wie im Code
    
    # Y_OFFSET aus der Konfiguration
    y_offset = -37.5  # mm
    translation_vector = np.array([0, y_offset, 0])
    
    # Verschiedene Stepper-Winkel simulieren
    stepper_angles = [0, 45, 90, 135, 180]  # Grad
    
    results_wrong = []  # Translation NACH Rotation (falsch)
    results_correct = []  # Translation VOR Rotation (korrekt)
    
    for angle_deg in stepper_angles:
        rotation = R.from_euler('z', angle_deg, degrees=True)
        
        # FALSCHE Methode: Translation nach Rotation
        rotated_wrong = rotation.apply(points3d) + translation_vector
        results_wrong.append(rotated_wrong)
        
        # KORREKTE Methode: Translation vor Rotation
        translated_first = points3d + translation_vector
        rotated_correct = rotation.apply(translated_first)
        results_correct.append(rotated_correct)
    
    print("📊 ANALYSE DER WAND-GERADHEIT:")
    print()
    
    for i, angle in enumerate(stepper_angles):
        wrong_points = results_wrong[i]
        correct_points = results_correct[i]
        
        # Berechne Krümmung (Standardabweichung der Abstände vom Zentrum)
        center_wrong = np.mean(wrong_points[:, :2], axis=0)
        center_correct = np.mean(correct_points[:, :2], axis=0)
        
        distances_wrong = np.linalg.norm(wrong_points[:, :2] - center_wrong, axis=1)
        distances_correct = np.linalg.norm(correct_points[:, :2] - center_correct, axis=1)
        
        curvature_wrong = np.std(distances_wrong)
        curvature_correct = np.std(distances_correct)
        
        print(f"Stepper-Winkel {angle:3d}°:")
        print(f"  Falsche Methode - Krümmung: {curvature_wrong:6.1f}mm")
        print(f"  Korrekte Methode - Krümmung: {curvature_correct:6.1f}mm")
        print(f"  Verbesserung: {((curvature_wrong - curvature_correct)/curvature_wrong)*100:4.1f}%")
        print()
    
    # Berechne Y_OFFSET-Verhalten
    print("🔄 Y_OFFSET-VERHALTEN:")
    print()
    
    # Verfolge einen spezifischen Punkt durch die Rotation
    test_point = np.array([1000, 0, 0])  # 1m geradeaus
    
    for i, angle in enumerate(stepper_angles):
        rotation = R.from_euler('z', angle, degrees=True)
        
        # Falsche Methode
        rotated_wrong = rotation.apply(test_point) + translation_vector
        
        # Korrekte Methode  
        translated_first = test_point + translation_vector
        rotated_correct = rotation.apply(translated_first)
        
        print(f"Stepper-Winkel {angle:3d}°:")
        print(f"  Falsch: Y_OFFSET bleibt bei ({translation_vector[0]:5.1f}, {translation_vector[1]:5.1f}, {translation_vector[2]:5.1f})")
        print(f"          Punkt wird zu: ({rotated_wrong[0]:6.1f}, {rotated_wrong[1]:6.1f}, {rotated_wrong[2]:6.1f})")
        print(f"  Korrekt: Y_OFFSET rotiert mit zu: ({(rotation.apply(translation_vector))[0]:6.1f}, {(rotation.apply(translation_vector))[1]:6.1f}, {(rotation.apply(translation_vector))[2]:6.1f})")
        print(f"           Punkt wird zu: ({rotated_correct[0]:6.1f}, {rotated_correct[1]:6.1f}, {rotated_correct[2]:6.1f})")
        print()
    
    print("✅ FAZIT:")
    print("   Bei der FALSCHEN Methode bleibt der Y_OFFSET in Weltkoordinaten fixiert")
    print("   → Alle vertikalen Flächen werden tangential verschoben")
    print("   → Gerade Wände erscheinen als Bögen")
    print()
    print("   Bei der KORREKTEN Methode rotiert der Y_OFFSET mit dem LiDAR")
    print("   → Der LiDAR-Versatz bleibt relativ zur aktuellen Orientierung")
    print("   → Wände bleiben gerade")
    
    return results_wrong, results_correct

if __name__ == "__main__":
    demonstrate_rotation_order()