#!/usr/bin/env python3
"""
Demonstriert Z-Achsen-Konfiguration für CloudCompare-Kompatibilität
"""

from lib.config import Config
from lib.pointcloud import process_raw
import numpy as np
import json

def test_z_configurations(scan_id='250924-1743'):
    """Teste verschiedene Z-Achsen-Konfigurationen"""
    
    print("🔧 Z-ACHSEN-KONFIGURATIONS-TEST")
    print("="*50)
    
    # Lade Konfiguration
    config = Config()
    config.init(scan_id)
    config.set(True, 'ENABLE_3D')
    config.set(False, 'ENABLE_VERTEXCOLOUR')
    
    print(f"📁 Verwende Scan: {scan_id}")
    
    # Test 1: Standard-Koordinaten
    print("\n1️⃣  STANDARD-KOORDINATEN:")
    print("   INVERT_Z_AXIS: false")
    print("   ➜ Z-Achse kann nach unten zeigen (CloudCompare Problem)")
    
    results_normal = process_raw(config, save=False)
    if results_normal['intensity']:
        z_coords = results_normal['intensity'].points[:, 2]
        print(f"   Z-Bereich: {z_coords.min():.3f} bis {z_coords.max():.3f}")
        print(f"   Z-Mittelwert: {z_coords.mean():.3f}")
    
    # Test 2: Temporäre Z-Inversion für Demonstration
    print("\n2️⃣  CLOUDCOMPARE-KOMPATIBLE KOORDINATEN:")
    print("   INVERT_Z_AXIS: true (simuliert)")
    print("   ➜ Z-Achse zeigt nach oben")
    
    # Simuliere Z-Inversion
    if results_normal['intensity']:
        inverted_points = results_normal['intensity'].points.copy()
        inverted_points[:, 2] *= -1  # Z-Achse invertieren
        z_coords_inv = inverted_points[:, 2]
        print(f"   Z-Bereich: {z_coords_inv.min():.3f} bis {z_coords_inv.max():.3f}")
        print(f"   Z-Mittelwert: {z_coords_inv.mean():.3f}")
        
        # Zeige Unterschied
        print(f"   📊 Differenz: Z-Werte um {abs(z_coords.mean() - z_coords_inv.mean()):.3f} verschoben")
    
    print("\n" + "="*50)
    print("🎯 ANLEITUNG FÜR CLOUDCOMPARE:")
    print("="*50)
    print("1. Öffnen Sie config.json")
    print("2. Ändern Sie in der '3D' Sektion:")
    print('   "INVERT_Z_AXIS": true,')
    print('   "CLOUDCOMPARE_COMPATIBLE": true')
    print("3. Führen Sie einen neuen Scan durch oder verarbeiten Sie bestehende Daten neu")
    print("4. Die positive Z-Achse zeigt dann in CloudCompare korrekt nach oben!")
    
    print("\n💡 ZUSÄTZLICHE OPTIONEN:")
    print("   UP_VECTOR: [0, 0, 1]  # Standard Z-up")
    print("   UP_VECTOR: [0, 1, 0]  # Y-up (manche CAD-Programme)")
    print("   UP_VECTOR: [0, 0, -1] # Z-down (invertiert)")

if __name__ == "__main__":
    test_z_configurations()
