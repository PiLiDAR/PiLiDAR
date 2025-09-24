#!/bin/bash
cd /home/pi/PiLIDAR-Pi5 || exit 1
source pilidar_env/bin/activate 2>/dev/null || true
exec python3 PiLiDAR.py --gui
