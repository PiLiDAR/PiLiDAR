'''
Pre-flight hardware diagnostics ("pilidar doctor").

Roadmap Phase 1 deliverable: an automated diagnostic routine checking UART
communication, I2C IMU connectivity, camera availability, and storage space
before beginning a scan.

Run it on the Pi before a scan to catch a dead LiDAR, a loose I2C jumper, a
camera that failed to enumerate, or a full disk:

    ~/venv/bin/python lib/diagnostics.py
    ~/venv/bin/python -m lib.diagnostics --json
    ~/venv/bin/python -m lib.diagnostics --lidar-seconds 5

Exit code is 0 when nothing FAILs (WARN/SKIP are non-fatal), 1 otherwise, so it
can gate a scan in a shell script or systemd unit. The ``run_doctor()`` API
returns the same result list for programmatic use (the Phase 3 ``pilidar`` CLI
will call it).
'''

import argparse
import importlib
import json
import os
import shutil
import sys

try:
    from lib.config import Config
    from lib.platform_utils import get_platform
except ImportError:
    from config import Config
    from platform_utils import get_platform


# status values
OK = "OK"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"

# minimum free space before a scan (bytes): warn below WARN, fail below FAIL
DISK_WARN_BYTES = 1 * 1024**3     # 1 GiB
DISK_FAIL_BYTES = 200 * 1024**2   # 200 MiB


class Result:
    def __init__(self, name, status, message):
        self.name = name
        self.status = status
        self.message = message

    def as_dict(self):
        return {"check": self.name, "status": self.status, "message": self.message}


# ---------------------------------------------------------------------------
# individual checks
# ---------------------------------------------------------------------------

def check_python(config):
    version = sys.version_info
    if version >= (3, 11):
        return OK, f"Python {version.major}.{version.minor}.{version.micro} ({sys.executable})"
    return FAIL, f"Python {version.major}.{version.minor} < 3.11 (requires-python >=3.11)"


def check_platform(config):
    platform = get_platform()
    if platform == "RaspberryPi":
        return OK, f"platform: {platform}"
    return WARN, f"platform: {platform} (hardware checks will be skipped)"


def check_storage(config):
    root = config.scans_root
    target = root if os.path.isdir(root) else os.path.dirname(root) or "."
    try:
        usage = shutil.disk_usage(target)
    except OSError as e:
        return FAIL, f"cannot stat {target}: {e}"

    free = usage.free
    free_gib = free / 1024**3
    msg = f"{free_gib:.1f} GiB free on {target}"
    if free < DISK_FAIL_BYTES:
        return FAIL, msg
    if free < DISK_WARN_BYTES:
        return WARN, msg
    return OK, msg


def check_hugin(config):
    tools = ["hugin_executor", "cpfind", "cpclean", "linefind", "autooptimiser"]
    on_path = [t for t in tools if shutil.which(t)]
    missing = [t for t in tools if not shutil.which(t)]

    if not missing:
        return OK, "hugin tools in PATH: " + ", ".join(tools)

    # mirror lib/pano_utils.py: fall back to the Flatpak build
    if shutil.which("flatpak"):
        return WARN, "not in PATH, using Flatpak fallback (missing: " + ", ".join(missing) + ")"
    if len(on_path) == 0:
        return FAIL, "no hugin tools found in PATH and no Flatpak (needed for panorama stitching)"
    return WARN, "missing: " + ", ".join(missing)


def check_lidar(config, seconds=3.0):
    """Open the LiDAR UART and verify a packet stream (device present + spinning)."""
    try:
        from lib.lidar_driver import Lidar
    except ImportError:
        from lidar_driver import Lidar  # type: ignore

    import time

    try:
        lidar = Lidar(config, visualization=None)
    except Exception as e:
        return FAIL, f"cannot open {config.PORT}: {e.__class__.__name__}: {e}"

    try:
        lidar.start_reader()
        t0 = time.time()
        packets = 0
        while time.time() - t0 < seconds:
            if lidar.read_from_queue(timeout=0.5):
                packets += 1
        elapsed = time.time() - t0
        lidar.stop_reader()
        dropped = lidar.dropped_packets
        malformed = lidar.malformed_packets
    finally:
        try:
            lidar.serial_connection.close()
        except Exception:
            pass

    rate = packets / elapsed if elapsed > 0 else 0.0
    if packets == 0:
        return FAIL, f"{config.PORT} open but no packets in {elapsed:.1f}s (LiDAR powered/spinning?)"
    msg = f"{rate:.0f} packets/s ({rate * lidar.dlength:.0f} samples/s), {dropped} dropped, {malformed} malformed"
    if dropped or malformed:
        return WARN, msg
    return OK, msg


def check_imu(config):
    bus_no = config.get("IMU", "i2c_bus")
    address = config.get("IMU", "device_address")
    try:
        import smbus
    except ImportError:
        return SKIP, "smbus not installed"

    try:
        bus = smbus.SMBus(bus_no)
    except (OSError, FileNotFoundError) as e:
        return FAIL, f"/dev/i2c-{bus_no} not available: {e}"

    try:
        who_am_i = bus.read_byte_data(address, 0x75)  # MPU6050_RA_WHO_AM_I
    except OSError as e:
        return FAIL, f"no I2C response from 0x{address:02x} on bus {bus_no}: {e}"
    finally:
        bus.close()

    if who_am_i in (0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x98):
        return OK, f"MPU6050 on i2c-{bus_no} @ 0x{address:02x} (WHO_AM_I=0x{who_am_i:02x})"
    return WARN, f"unexpected WHO_AM_I=0x{who_am_i:02x} on i2c-{bus_no} @ 0x{address:02x}"


def check_camera(config):
    try:
        picamera2 = importlib.import_module("picamera2")
    except ImportError:
        return SKIP, "picamera2 not installed"

    try:
        info = picamera2.Picamera2.global_camera_info()
    except Exception as e:
        return FAIL, f"camera enumeration failed: {e.__class__.__name__}: {e}"

    if not info:
        return FAIL, "no camera detected (ribbon connected / enabled?)"
    models = [c.get("Model", "?") for c in info]
    return OK, f"{len(info)} camera(s): " + ", ".join(models)


def check_gpio(config):
    try:
        import lgpio
    except ImportError:
        return SKIP, "lgpio not installed"

    handle = lgpio.gpiochip_open(0)
    if handle < 0:
        return FAIL, f"cannot open gpiochip 0 ({lgpio.error_text(handle)})"
    lgpio.gpiochip_close(handle)
    return OK, "gpiochip 0 open"


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def run_doctor(config=None, lidar_seconds=3.0, skip=()):
    """Run all checks and return a list of :class:`Result`."""
    if config is None:
        # Config.__init__ already sets scans_root / device / PORT; init() would
        # create an empty scan directory, which diagnostics must not litter.
        # Lidar.__init__ reads config.raw_path (normally set by init()); it is
        # unused for the read-only packet check, so provide a placeholder.
        config = Config()
        config.raw_path = None

    platform = get_platform()
    on_pi = platform == "RaspberryPi"

    results = []
    results.append(Result("python", *check_python(config)))
    results.append(Result("platform", *check_platform(config)))
    results.append(Result("storage", *check_storage(config)))
    results.append(Result("hugin", *check_hugin(config)))

    # hardware checks only make sense on the Pi; SKIP them elsewhere
    if on_pi:
        hardware = [
            ("lidar", lambda: check_lidar(config, seconds=lidar_seconds)),
            ("imu", lambda: check_imu(config)),
            ("camera", lambda: check_camera(config)),
            ("gpio", lambda: check_gpio(config)),
        ]
    else:
        hardware = [(name, None) for name in ("lidar", "imu", "camera", "gpio")]

    for name, fn in hardware:
        if name in skip:
            results.append(Result(name, SKIP, "skipped"))
        elif fn is None:
            results.append(Result(name, SKIP, "not a Raspberry Pi"))
        else:
            results.append(Result(name, *fn()))

    return results


def _print(results, use_json=False):
    if use_json:
        print(json.dumps([r.as_dict() for r in results], indent=2))
        return
    print("PiLiDAR hardware diagnostics")
    print("=" * 60)
    for r in results:
        print(f"[{r.status:4}] {r.name:<9} {r.message}")
    print("=" * 60)
    counts = {s: sum(1 for r in results if r.status == s) for s in (OK, WARN, FAIL, SKIP)}
    print(f"{counts[OK]} ok, {counts[WARN]} warn, {counts[FAIL]} fail, {counts[SKIP]} skipped")


def main(argv=None):
    parser = argparse.ArgumentParser(description="PiLiDAR pre-flight hardware diagnostics")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument("--lidar-seconds", type=float, default=3.0,
                        help="seconds to sample the LiDAR UART (default 3)")
    parser.add_argument("--skip", nargs="*", default=[],
                        help="check names to skip: lidar imu camera gpio")
    args = parser.parse_args(argv)

    config = Config()
    results = run_doctor(config, lidar_seconds=args.lidar_seconds, skip=args.skip)
    _print(results, use_json=args.json)

    return 1 if any(r.status == FAIL for r in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
