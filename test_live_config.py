#!/usr/bin/env python3
"""Test der Live-Konfiguration mit Ihren Werten"""

from lib.config import Config

def test_live_config():
    print("=== Test der Live-Konfiguration ===\n")
    
    # Lade die aktuelle Konfiguration
    config = Config('/home/pi/PiLIDAR-Pi5/config.json')
    
    print(f"📋 Aktuelle Konfiguration:")
    print(f"   SCAN_ANGLE: {config.SCAN_ANGLE}°")
    print(f"   TARGET_RES: {config.TARGET_RES}")
    print(f"   target_res: {config.target_res}°")
    print(f"   h_res: {config.h_res}°")
    print(f"   horizontal_steps: {config.horizontal_steps}")
    
    # Simuliere GUI-Update mit Ihren Werten
    print(f"\n🔄 Simuliere GUI-Update:")
    h_res_new = 0.1666666667
    scan_angle_new = 1.9
    
    print(f"   Setze TARGET_RES = {h_res_new}")
    print(f"   Setze SCAN_ANGLE = {scan_angle_new}")
    
    # Update die Konfiguration
    config.set(scan_angle_new, "STEPPER", "SCAN_ANGLE")
    config.SCAN_ANGLE = scan_angle_new
    config.set(h_res_new, "LIDAR", "TARGET_RES")
    config.update_target_res(h_res_new)
    
    print(f"\n📊 Nach Update:")
    print(f"   SCAN_ANGLE: {config.SCAN_ANGLE}°")
    print(f"   TARGET_RES: {config.TARGET_RES}")
    print(f"   target_res: {config.target_res}°")
    print(f"   h_res: {config.h_res}°")
    print(f"   horizontal_steps: {config.horizontal_steps}")
    print(f"   steps: {config.steps}")
    print(f"   max_packages: {config.max_packages}")
    
    # Berechne erwartete Ebenen
    expected = scan_angle_new / h_res_new
    print(f"\n🎯 Vergleich:")
    print(f"   Mathematisch erwartet: {expected:.1f} Ebenen")
    print(f"   Config berechnet: {config.horizontal_steps} Ebenen")
    
    if config.horizontal_steps != 5:
        print(f"   ⚠️  Sie berichteten 5 Ebenen, aber Config berechnet {config.horizontal_steps}")
        print(f"   Möglicherweise wird eine andere Berechnung verwendet!")

if __name__ == "__main__":
    test_live_config()