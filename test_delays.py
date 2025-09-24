#!/usr/bin/env python3
"""
Vergleichstest: PWM vs. traditionelle Delays
Demonstriert den Unterschied zwischen PWM-Steuerung und traditionellen STEP_DELAY/SCAN_DELAY
"""

import time
from lib.config import Config

def test_delay_configuration():
    """Teste die Delay-Konfiguration"""
    
    print("=== VERGLEICH: PWM vs. TRADITIONELLE DELAYS ===")
    
    config = Config()
    config.init(scan_id='delay_test')
    
    # Aktuelle Konfiguration anzeigen
    use_pwm = config.get("STEPPER", "USE_PWM", default=False)
    step_delay = config.get("STEPPER", "STEP_DELAY")
    scan_delay = config.get("STEPPER", "SCAN_DELAY")
    
    print(f"\n📊 AKTUELLE KONFIGURATION:")
    print(f"  USE_PWM: {use_pwm}")
    print(f"  STEP_DELAY: {step_delay:.6f}s ({step_delay*1000:.2f}ms)")
    print(f"  SCAN_DELAY: {scan_delay:.6f}s ({scan_delay*1000:.2f}ms)")
    
    # Berechne theoretische Geschwindigkeiten
    if not use_pwm:
        print("\n🐢 TRADITIONELLE DELAY-STEUERUNG:")
        print("  ✅ STEP_DELAY wird zwischen jedem Motorschritt verwendet")
        print("  ✅ SCAN_DELAY wird nach jeder Stepper-Position verwendet")
        print("  📏 Präzise Timing-Kontrolle")
        print("  ⏱️  Langsamerer Scan (höhere Präzision)")
        
        # Berechne Scan-Zeit für 180°
        microsteps = config.get("STEPPER", "MICROSTEPS")
        stepper_res = config.get("STEPPER", "STEPPER_RES")  
        gear_ratio = config.get("STEPPER", "GEAR_RATIO")
        
        if isinstance(gear_ratio, str):
            gear_ratio = eval(gear_ratio)  # "1 + 38/14" → 3.714
            
        steps_per_degree = (stepper_res * microsteps * gear_ratio) / 360
        steps_for_180 = steps_per_degree * 180
        
        step_time = step_delay * steps_for_180
        
        # Anzahl Scan-Positionen (abhängig von TARGET_RES)  
        target_res = config.get("LIDAR", "TARGET_RES")
        if isinstance(target_res, str) and "/" in target_res:
            num, den = target_res.split("/")
            resolution_deg = float(num) / float(den)
        else:
            resolution_deg = float(target_res)
            
        scan_positions = int(180 / resolution_deg)
        scan_time = scan_delay * scan_positions
        
        total_time = step_time + scan_time
        
        print(f"  📊 Geschätzter 180°-Scan:")
        print(f"     Schrittzeit: {step_time:.1f}s")
        print(f"     Scan-Wartezeit: {scan_time:.1f}s") 
        print(f"     Gesamtzeit: {total_time:.1f}s ({total_time/60:.1f}min)")
        
    else:
        print("\n🚀 PWM-STEUERUNG:")
        print("  ⚠️  STEP_DELAY und SCAN_DELAY werden UMGANGEN")
        print("  ⚡ Schnellere Bewegung durch PWM-Frequenz")
        print("  📊 Weniger präzise Timing-Kontrolle")
        print("  ⏱️  Schnellerer Scan (geringere Präzision)")
    
    print("\n💡 EMPFEHLUNG:")
    if not use_pwm:
        print("  Für maximale Präzision: Aktuelle Einstellung beibehalten")
        print("  Für höhere Geschwindigkeit: USE_PWM auf true setzen")
    else:
        print("  Aktuelle Einstellung: Schnelle Scans")
        print("  Für höhere Präzision: USE_PWM auf false setzen")
    
    print(f"\n🔧 ÄNDERUNG DER KONFIGURATION:")
    print(f"  In config.json unter STEPPER:")
    print(f"    \"USE_PWM\": {str(not use_pwm).lower()}")
    print(f"    \"STEP_DELAY\": {step_delay}")
    print(f"    \"SCAN_DELAY\": {scan_delay}")

if __name__ == "__main__":
    test_delay_configuration()