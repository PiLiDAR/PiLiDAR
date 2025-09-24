
import threading
import serial
import time
from collections import deque

class SerialIO:
    def __init__(self, port="/dev/ttyUSB0", baud=921600, read_chunk=1024, max_buffer=65536):
        self.port = port
        self.baud = baud
        self.read_chunk = read_chunk
        self.max_buffer = max_buffer
        self._ser = None
        self._buf = deque(maxlen=max_buffer)
        self._thread = None
        self._stop = threading.Event()

    def open(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=0)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while not self._stop.is_set():
            if self._ser is None: 
                time.sleep(0.01)
                continue
            data = self._ser.read(self.read_chunk)
            if data:
                self._buf.extend(data)
            else:
                time.sleep(0.001)

    def read_bytes(self, n):
        out = bytearray()
        while len(out) < n and self._buf:
            out.append(self._buf.popleft())
        return bytes(out)

    def available(self):
        return len(self._buf)

    def write_bytes(self, data: bytes) -> None:
        if self._ser is None:
            return
        try:
            self._ser.write(data)
        except Exception:  # pragma: no cover - serial port failures
            pass

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self._thread = None
        self._buf.clear()
