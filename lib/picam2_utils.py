"""
Picamera2-based camera capture for the PiLiDAR panorama rig.

Drop-in replacement for ``lib/rpicam_utils.py`` (which shells out to
``rpicam-still``). Instead of spawning a new camera process per frame, this
module drives the sensor through a single persistent Picamera2 session:

    1. configure still + preview streams and start the camera
    2. let the 3A (AE/AWB) pipeline settle over a few frames
    3. read the converged ``ExposureTime`` / ``AnalogueGain`` / ``ColourGains``
       from ``capture_metadata()``
    4. lock 3A (``AeEnable=False``, ``AwbEnable=False``) and drive the sensor
       manually for the HDR bracket

HDR bracketing keeps the same shutter-for-dark / gain-for-bright strategy as
the rpicam path: negative EV steps lengthen the exposure (shutter), positive EV
steps raise the gain. This avoids motion blur on the bright frames while still
recovering shadow detail on the dark frames.

Units
-----
Picamera2 reports ``ExposureTime`` in **nanoseconds** and ``AnalogueGain`` /
``DigitalGain`` as linear multipliers. The rest of the project works in
**seconds** for exposure and total gain (ISO/100), so conversions happen at the
boundary in this module.

Only importable/runnable on the target Raspberry Pi (needs the camera stack);
on a desktop the import of ``picamera2`` will fail and the caller should fall
back to ``lib/rpicam_utils.py``.
"""
import os
import time

# IMX477 (Raspberry Pi HQ Camera) max analogue gain. Bracket steps that would
# exceed this are clamped here (DigitalGain is kept at 1 per the roadmap).
MAX_ANALOGUE_GAIN = 16.0


def _ns(seconds):
    """Convert an exposure time in seconds to Picamera2 nanoseconds."""
    return int(round(seconds * 1e9))


class Camera:
    """A persistent Picamera2 session for the panorama rig.

    Usage::

        with Camera() as cam:
            exposure_time, gain, awbgains = cam.estimate_parameters(config)
            for angle in angles:
                cam.take_hdr(path, exposure_time, gain, awbgains, AEB, AEB_stops)
    """

    def __init__(self):
        # Imported lazily so the module can be imported (and syntax-checked) on
        # a desktop without the camera stack; only instantiation needs picamera2.
        from picamera2 import Picamera2
        self._Picamera2 = Picamera2

    # -- lifecycle -----------------------------------------------------------
    def __enter__(self):
        self.camera = self._Picamera2()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.camera.stop()
        except Exception:
            pass
        return False

    # -- configuration -------------------------------------------------------
    def _configure(self, still_dims, preview_dims=None):
        """Configure still + (optional) preview streams and start the camera."""
        configs = {"main": {"size": tuple(still_dims), "format": "bgr8"}}
        if preview_dims:
            configs["lores"] = {"size": tuple(preview_dims), "format": "bgr8"}
        self.camera.configure(configs)
        self.camera.start()

    def _settle(self, frames=10, interval=0.1):
        """Let the 3A pipeline converge by capturing a few frames."""
        for _ in range(frames):
            self.camera.capture_metadata()
            time.sleep(interval)

    def _lock_3a(self):
        """Freeze AE and AWB so subsequent frames are fully manual."""
        self.camera.set_controls({"AeEnable": False, "AwbEnable": False})

    def _set_manual(self, exposure_time, gain, awbgains):
        """Apply manual exposure / gain / white-balance to the sensor.

        ``exposure_time`` is in seconds, ``gain`` is the total linear gain, and
        ``awbgains`` is ``[R, B]``. The total gain is split into analogue (capped
        at ``MAX_ANALOGUE_GAIN``) and digital components.
        """
        analogue_gain = min(gain, MAX_ANALOGUE_GAIN)
        digital_gain = 1.0 if gain <= MAX_ANALOGUE_GAIN else gain / MAX_ANALOGUE_GAIN
        self.camera.set_controls({
            "ExposureTime": _ns(exposure_time),
            "AnalogueGain": analogue_gain,
            "DigitalGain": digital_gain,
            "ColourGains": [awbgains[0], awbgains[1]],
        })

    # -- calibration ---------------------------------------------------------
    def estimate_parameters(self, config):
        """Converge 3A, read the result, and lock it.

        Returns ``(exposure_time, gain, awbgains)`` in the same units as
        ``rpicam_utils.estimate_camera_parameters`` (seconds, total gain, [R, B]).
        """
        preview_dims = config.get("CAM", "preview_dims")
        set_gain = config.get("CAM", "set_gain")

        self._configure(still_dims=preview_dims)
        self._settle()

        meta = self.camera.capture_metadata()
        exposure_time = meta["ExposureTime"] / 1e9          # ns -> s
        gain = meta["AnalogueGain"] * meta["DigitalGain"]   # total linear gain
        awbgains = list(meta["ColourGains"])                # [R, B]

        self._lock_3a()

        # Optionally pin the gain and recompute the exposure to keep the same
        # overall exposure (exposure is inversely proportional to gain).
        if set_gain:
            gain = set_gain
            exposure_time = exposure_time * (gain / (meta["AnalogueGain"] * meta["DigitalGain"]))

        return exposure_time, gain, awbgains

    # -- capture -------------------------------------------------------------
    def take_photo(self, path, dims, exposure_time=None, gain=None,
                   awbgains=None, blocking=True):
        """Capture a single still at the given manual parameters."""
        self._configure(still_dims=dims)
        if exposure_time is not None and gain is not None:
            self._lock_3a()
            self._set_manual(exposure_time, gain, awbgains or [1.0, 1.0])
        self.camera.capture_file(path)
        return path

    def take_hdr(self, path, AEB=1, AEB_stops=2, dims=(4056, 3040),
                 exposure_time=None, gain=None, awbgains=None, blocking=True):
        """Capture an HDR bracket using shutter-for-dark / gain-for-bright.

        Mirrors ``rpicam_utils.take_HDR_photo``: negative EV steps lengthen the
        exposure, positive EV steps raise the gain. Filenames carry the EV offset
        (``..._<ev>.jpg``) so the Hugin reorder step can match them to the
        template's per-angle EV order.
        """
        path_split = os.path.splitext(path)
        AEB = int(AEB)
        stop_multiplier = 2 ** AEB_stops
        paths = []

        self._configure(still_dims=dims)
        self._lock_3a()

        for step in range(-AEB, AEB + 1):
            if step < 0:
                hdr_exposure_time = exposure_time * (stop_multiplier ** step)
                hdr_gain = gain
            else:
                hdr_exposure_time = exposure_time
                hdr_gain = gain * (stop_multiplier ** step)

            self._set_manual(hdr_exposure_time, hdr_gain, awbgains or [1.0, 1.0])
            hdr_path = f"{path_split[0]}_{step * AEB_stops}{path_split[1]}"
            self.camera.capture_file(hdr_path)
            paths.append(hdr_path)

        return paths


# ---------------------------------------------------------------------------
# Module-level drop-in wrappers (match the rpicam_utils interface so PiLiDAR.py
# can fall back between the two backends without changing its call sites).
# Each call opens and closes its own session; for maximum throughput use the
# ``Camera`` context manager directly and keep one session across all angles.
# ---------------------------------------------------------------------------
def estimate_camera_parameters(config):
    with Camera() as cam:
        return cam.estimate_parameters(config)


def take_photo(path, dims=(4056, 3040), exposure_time=None, gain=None,
               awbgains=None, blocking=True, **_ignored):
    with Camera() as cam:
        return cam.take_photo(path, dims, exposure_time, gain, awbgains, blocking)


def take_HDR_photo(path, AEB=1, AEB_stops=2, dims=(4056, 3040),
                   exposure_time=None, gain=None, awbgains=None,
                   denoise="cdn_hq", awb="auto", sharpness=0.5, saturation=0.7,
                   save_raw=False, blocking=True):
    with Camera() as cam:
        return cam.take_hdr(path, AEB, AEB_stops, dims, exposure_time, gain,
                            awbgains, blocking)
