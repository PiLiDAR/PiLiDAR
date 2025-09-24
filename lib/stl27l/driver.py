
import time
from typing import List, Optional, Callable
from .io import SerialIO
from .protocol import LiPkgParser

class STL27LDriver:
    """Lightweight high-level driver (SDK-like API)."""
    
    def __init__(self, port="/dev/ttyUSB0", baud=921600):
        self.port = port
        self.baud = baud
        self.io = SerialIO(port=port, baud=baud)
        self.parser = LiPkgParser(measure_point_hz=21600)
        self._running = False
        self._timestamp_cb: Optional[Callable[[], int]] = None

    def register_timestamp(self, cb: Callable[[], int]):
        self._timestamp_cb = cb

    def start(self):
        if self._running:
            return True
        self.io.open()
        self._running = True
        return True

    def wait_connect(self, timeout_ms=2000) -> bool:
        t_end = time.time() + timeout_ms / 1000.0
        while time.time() < t_end:
            if self.io.available() >= 64:
                chunk = self.io.read_bytes(self.io.available())
                self.parser.feed(chunk)
                pkt = self.parser.next_packet()
                if pkt is not None:
                    return True
            time.sleep(0.005)
        return False

    def get_scan(self, timeout_ms=1000):
        """Return the next list of 12 points (or None on timeout)."""
        t_end = time.time() + timeout_ms / 1000.0
        while time.time() < t_end:
            if self.io.available():
                chunk = self.io.read_bytes(self.io.available())
                self.parser.feed(chunk)
                pkt = self.parser.next_packet()
                if pkt is not None:
                    return pkt
            time.sleep(0.001)
        return None

    def get_spin_hz(self) -> float:
        return self.parser.spin_deg_per_s / 360.0

    def get_spin_deg_per_s(self) -> float:
        """Get motor spin speed in degrees per second."""
        try:
            if hasattr(self.parser, 'spin_deg_per_s') and self.parser.spin_deg_per_s is not None:
                speed = float(self.parser.spin_deg_per_s)
                # Plausibilitätsprüfung: STL27L läuft normalerweise zwischen 300-1800°/s
                if 0 < speed <= 3600:
                    return speed
            # Fallback: Berechne aus get_spin_hz() falls verfügbar
            if hasattr(self.parser, 'spin_hz') and self.parser.spin_hz is not None:
                return float(self.parser.spin_hz * 360.0)
            # Standard STL27L Geschwindigkeit als Fallback (10 Hz = 600°/s)
            return 600.0
        except (AttributeError, TypeError, ValueError):
            return 600.0  # Sicherer Fallback-Wert für STL27L

    def get_motor_speed(self) -> float:
        """Get motor speed in Hz."""
        return float(self.parser.spin_deg_per_s)
    
    def _send_command(self, command: int) -> bool:
        """Send command to LiDAR using SDK-compatible protocol."""
        try:
            cmd_byte = bytes([command])
            self.io.write_bytes(cmd_byte)
            return True
        except Exception as e:
            print(f"Failed to send command 0x{command:02X}: {e}")
            return False

    def stop(self):
        self.io.close()
        self._running = False

    # Placeholders for vendor motor commands (if supported by your unit):
    def motor_start(self):
        """Start the LiDAR motor (according to official SDK, motor starts automatically with driver start)."""
        if not self._running:
            print("Driver not running - motor cannot start")
            return False
        # According to official SDK: Motor starts automatically when driver starts
        # No explicit motor start command is needed for STL27L
        print("STL27L motor starts automatically with driver - no explicit command needed")
        return True

    def motor_stop(self):
        """Stop the LiDAR motor (according to official SDK, motor stops when driver stops)."""
        # According to official SDK: Motor stops automatically when driver stops
        # No explicit motor stop command exists for STL27L
        print("STL27L motor stops automatically when driver stops - stopping driver...")
        return self.stop()
