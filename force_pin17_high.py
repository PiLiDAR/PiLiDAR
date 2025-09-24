#!/usr/bin/env python3
"""Utility to ensure GPIO pin 17 is always HIGH to keep stepper motor disabled.

This script can be run independently or imported as a module to force
pin 17 to HIGH state, ensuring the stepper motor remains disabled.
"""

import os
import sys

def force_pin17_high():
    """Force GPIO pin 17 to HIGH state."""
    try:
        # Set environment variable for RPi.GPIO compatibility
        os.environ.setdefault("RPI_LGPIO_REVISION", "0xa020d3")
        import RPi.GPIO as GPIO
        
        # Configure GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(17, GPIO.OUT)
        GPIO.output(17, GPIO.HIGH)
        
        return True
    except Exception as e:
        print(f"Error setting GPIO pin 17: {e}")
        return False

def ensure_pin17_high_on_boot():
    """Set up pin 17 to be HIGH on system boot."""
    success = force_pin17_high()
    if success:
        print("✓ GPIO pin 17 set to HIGH - Stepper motor disabled")
    else:
        print("✗ Failed to set GPIO pin 17 to HIGH")
    return success

if __name__ == "__main__":
    print("Forcing GPIO pin 17 to HIGH to disable stepper motor...")
    success = ensure_pin17_high_on_boot()
    sys.exit(0 if success else 1)