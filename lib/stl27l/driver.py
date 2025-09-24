
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
        return float(self.parser.spin_deg_per_s)

    def stop(self):
        self.io.close()
        self._running = False

    # Placeholders for vendor motor commands (if supported by your unit):
    def motor_start(self):
        """Send vendor-specific UART command to start motor (if available)."""
        # Example (pseudo): self.io._ser.write(b"...")
        pass

    def motor_stop(self):
        """Send vendor-specific UART command to stop motor (if available)."""
        try:
            if hasattr(self.io, '_ser') and self.io._ser and self.io._ser.is_open:
                self.io._ser.write(b"0")  # Send motor stop command
                self.io._ser.flush()
        except Exception:
            pass  # Ignore errors during shutdown
