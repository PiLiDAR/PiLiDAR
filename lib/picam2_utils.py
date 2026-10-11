"""
Picamera2-based camera capture for the PiLiDAR panorama rig.

Drives the IMX477 through a single persistent Picamera2 session:

    1. configure the still stream and start the camera
    2. let the 3A (AE/AWB) pipeline settle over a few frames
    3. read the converged ExposureTime / AnalogueGain / ColourGains from
       capture_metadata()
    4. lock 3A (AeEnable=False, AwbEnable=False) and drive the sensor manually
       for the HDR bracket

HDR bracketing uses shutter-for-dark / gain-for-bright: negative EV steps
lengthen the exposure, positive EV steps raise the gain. This avoids motion
blur on the bright frames while still recovering shadow detail on the dark
frames. The IMX477 via libcamera exposes only AnalogueGain (no DigitalGain
control), so bracket gain is clamped to MAX_ANALOGUE_GAIN.

Units
-----
Picamera2 reports ExposureTime in nanoseconds and AnalogueGain as a linear
multiplier. The rest of the project works in seconds for exposure, so the
conversion happens at the boundary in this module.

Only runnable on the target Raspberry Pi (needs the libcamera camera stack).
"""
import os
import time

# IMX477 (Raspberry Pi HQ Camera) max analogue gain. libcamera exposes only
# AnalogueGain (no DigitalGain control), so bracket steps that would exceed this
# are clamped here.
MAX_ANALOGUE_GAIN = 16.0


def _ns(seconds):
    """Convert an exposure time in seconds to Picamera2 nanoseconds."""
    return int(round(seconds * 1e9))


class Camera:
    """A persistent Picamera2 session for the panorama rig.

    Usage::

        with Camera() as cam:
            exposure_time, gain, awbgains = cam.estimate_parameters(config)
            cam.configure(config.get("CAM", "dims"))   # full-resolution stills
            for angle in angles:
                cam.take_hdr(path, AEB, AEB_stops, exposure_time, gain, awbgains)
    """

    def __init__(self):
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
    def configure(self, still_dims, preview_dims=None):
        """Configure still + (optional) preview streams and start the camera.

        Uses ``create_still_configuration()`` so the full config structure
        (``raw``/``sensor``/``controls``/stream alignment) is built correctly;
        passing a bare dict to ``configure()`` raises ``KeyError: 'raw'``.
        Re-configuring requires the camera to be stopped first.
        """
        try:
            self.camera.stop()
        except Exception:
            pass
        config = self.camera.create_still_configuration(
            main={"size": tuple(still_dims)},
            lores={"size": tuple(preview_dims)} if preview_dims else None,
        )
        self.camera.configure(config)
        self.camera.start()

    def _settle(self, frames=10, interval=0.1):
        """Let the 3A pipeline converge by capturing a few frames."""
        for _ in range(frames):
            self.camera.capture_metadata()
            time.sleep(interval)

    def _lock_3a(self):
        """Freeze AE and AWB so subsequent frames are fully manual."""
        self.camera.set_controls({"AeEnable": False, "AwbEnable": False})

    def _flush(self):
        """Discard one frame so the next capture is not a still-in-flight
        auto-mode frame. After ``configure()`` (which restarts the camera in
        auto mode) + ``_lock_3a()``, the first queued frame can still carry the
        pre-lock 3A state; capturing and dropping it guarantees the following
        frame reflects the manual controls we set."""
        self.camera.capture_array()

    def _set_manual(self, exposure_time, gain, awbgains):
        """Apply manual exposure / gain / white-balance to the sensor.

        ``exposure_time`` is in seconds, ``gain`` is the total linear gain, and
        ``awbgains`` is ``[R, B]``. The IMX477 via libcamera only advertises
        ``AnalogueGain`` (no ``DigitalGain`` control), so the gain is clamped to
        ``MAX_ANALOGUE_GAIN``; bracket steps that would exceed it are capped.
        """
        analogue_gain = min(gain, MAX_ANALOGUE_GAIN)
        self.camera.set_controls({
            "ExposureTime": _ns(exposure_time),
            "AnalogueGain": analogue_gain,
            "ColourGains": [awbgains[0], awbgains[1]],
        })

    # -- calibration ---------------------------------------------------------
    def estimate_parameters(self, config):
        """Converge 3A at preview resolution, read the result, and lock it.

        Returns ``(exposure_time, gain, awbgains)`` in seconds / total linear
        gain / ``[R, B]``.
        """
        preview_dims = config.get("CAM", "preview_dims")
        set_gain = config.get("CAM", "set_gain")

        self.configure(still_dims=preview_dims)
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
    def take_photo(self, path, exposure_time, gain, awbgains):
        """Capture a single still at the given manual parameters.

        Assumes the camera is already configured (see ``configure()``).
        """
        self._lock_3a()
        self._set_manual(exposure_time, gain, awbgains)
        self._flush()  # drop the startup auto-mode frame
        self.camera.capture_file(path)
        return path

    def take_hdr(self, path, AEB, AEB_stops, exposure_time, gain, awbgains):
        """Capture an HDR bracket using shutter-for-dark / gain-for-bright.

        Negative EV steps lengthen the exposure, positive EV steps raise the
        gain. Filenames carry the EV offset (``..._<ev>.jpg``) so the Hugin
        reorder step can match them to the template's per-angle EV order.
        Assumes the camera is already configured at the still resolution.
        """
        path_split = os.path.splitext(path)
        AEB = int(AEB)
        stop_multiplier = 2 ** AEB_stops
        paths = []

        self._lock_3a()

        for step in range(-AEB, AEB + 1):
            if step < 0:
                hdr_exposure_time = exposure_time * (stop_multiplier ** step)
                hdr_gain = gain
            else:
                hdr_exposure_time = exposure_time
                hdr_gain = gain * (stop_multiplier ** step)

            self._set_manual(hdr_exposure_time, hdr_gain, awbgains)
            if step == -AEB:
                self._flush()  # drop the startup auto-mode frame once
            hdr_path = f"{path_split[0]}_{step * AEB_stops}{path_split[1]}"
            self.camera.capture_file(hdr_path)
            paths.append(hdr_path)

        return paths
