'''
Serial throughput benchmark for the high-speed LiDAR driver (v2.5.0).

Roadmap Phase 1: "Benchmark and validate sustained throughput across all
supported models: LD06, LD19, and STL27L."

Measures the sustained packet/sample throughput, drop rate and malformed
packet rate of the background reader thread + bounded queue at the configured
baud rate (STL27L: 921,600 baud -> ~1800 packets/s -> ~21,600 samples/s).

Usage:
    ~/venv/bin/python experiments/serial_benchmark.py [seconds]
'''

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lib.config import Config
from lib.lidar_driver import Lidar

# nominal packet rates (packets/s) at default motor speed
EXPECTED_RATE = {"LD06": 375, "LD19": 1800, "STL27L": 1800}


def main(duration=30):
    config = Config()
    config.init(scan_id="test")  # reuse scans/test, no new scan dir

    lidar = Lidar(config, visualization=None)
    expected = EXPECTED_RATE.get(config.DEVICE)
    print(f"device: {config.DEVICE} | port: {lidar.port} | baud: {config.get('LIDAR', config.DEVICE, 'BAUDRATE')}")
    print(f"queue size: {lidar.queue_size} | expected rate: ~{expected} packets/s")
    print(f"benchmarking for {duration} s ...")

    lidar.start_reader()
    t0 = time.time()
    packets = 0
    queue_high = 0
    try:
        while time.time() - t0 < duration:
            if lidar.read_from_queue(timeout=0.5):
                packets += 1
                lidar.out_i += 1
                if lidar.out_i >= lidar.out_len:
                    lidar.out_i = 0
            q = lidar.packet_queue.qsize()
            if q > queue_high:
                queue_high = q
    finally:
        elapsed = time.time() - t0
        lidar.stop_reader()
        lidar.serial_connection.close()

    rate = packets / elapsed
    dropped = lidar.dropped_packets
    malformed = lidar.malformed_packets

    print("-" * 60)
    print(f"elapsed:          {elapsed:8.1f} s")
    print(f"packets:          {packets:8d}   ({rate:8.1f} packets/s)")
    print(f"samples:          {packets * lidar.dlength:8d}   ({rate * lidar.dlength:8.1f} samples/s)")
    print(f"byte rate:        {rate * lidar.package_len:8.0f} B/s")
    print(f"queue high-water: {queue_high:8d} / {lidar.queue_size}")
    print(f"dropped packets:  {dropped:8d}")
    print(f"malformed:        {malformed:8d}")
    print("-" * 60)

    ok = dropped == 0
    if expected is not None:
        deviation = abs(rate - expected) / expected
        ok &= deviation < 0.10
        print(f"rate vs expected: {deviation*100:5.1f}% deviation from {expected} packets/s")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    raise SystemExit(main(duration))
