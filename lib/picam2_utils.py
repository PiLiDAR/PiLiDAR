"""
Picamera2-based camera capture for the PiLiDAR panorama rig.

Drives the IMX477 through a single persistent Picamera2 session:

    1. configure the still stream and start the camera
    2. let the 3A (AE/AWB) pipeline settle over a few frames
    3. read the converged ExposureTime / AnalogueGain / ColourGains from
       capture_metadata()
    4. lock 3A (AeEnable=False, AwbEnable=False) and drive the sensor manually
       for the HDR bracket

HDR bracketing is **shutter-only** at constant gain: the rig is stationary while
shooting the panorama, so every frame is a long exposure and there is no motion
blur to trade against — bracketing by shutter keeps the ISO (hence noise) low.
The gain is chosen as low as possible (ISO 100 preferred, ISO 200 cap) with the
brightness otherwise coming from exposure time.

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


def choose_gain_exposure(auto_exposure, auto_gain, base_gain=1.0, max_gain=2.0, max_exposure=1.0):
    """Pick the lowest ISO (gain) that keeps the exposure within ``max_exposure``.

    Brightness is taken from **shutter time, not gain**, to avoid ISO noise:
    ``base_gain`` (ISO 100) is preferred, and the gain is only raised — up to
    ``max_gain`` (ISO 200) — when the exposure would otherwise exceed
    ``max_exposure`` seconds. Returns ``(exposure_time_seconds, gain)``.
    """
    # exposure needed to keep the same light at the (low) base gain
    e_base = auto_exposure * auto_gain / base_gain
    if not max_exposure or e_base <= max_exposure:
        return e_base, base_gain
    # dark scene: raise gain to the cap to shorten the exposure
    gain = max(base_gain, max_gain)
    return auto_exposure * auto_gain / gain, gain


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

    def _sync_to(self, exposure_time, tolerance=0.02, attempts=12):
        """Block until the sensor reports the requested exposure.

        ``set_controls()`` only takes effect on the *next* captured frame, so a
        ``capture_file()`` issued right after it can still save the previous
        frame's exposure. This drains frames (via ``capture_metadata()``) until
        the reported ``ExposureTime`` matches ``exposure_time`` (within
        ``tolerance``, i.e. 2%, or 1 ms), so the subsequently saved frame really
        carries the exposure we asked for.
        """
        want_ns = _ns(exposure_time)
        slack = max(1_000_000, want_ns * tolerance)     # >= 1 ms or `tolerance`
        for _ in range(attempts):
            meta = self.camera.capture_metadata()
            if abs(meta["ExposureTime"] - want_ns) <= slack:
                return True
        return False

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
        base_gain = config.get("CAM", "base_gain")
        max_gain = config.get("CAM", "max_gain")
        max_exposure = config.get("CAM", "max_exposure")

        self.configure(still_dims=preview_dims)
        self._settle()

        meta = self.camera.capture_metadata()
        auto_exposure = meta["ExposureTime"] / 1e9               # ns -> s
        auto_gain = meta["AnalogueGain"] * meta["DigitalGain"]   # total linear gain
        awbgains = list(meta["ColourGains"])                     # [R, B]

        self._lock_3a()

        # Take brightness from shutter time, not gain, to avoid ISO noise: use the
        # lowest gain (ISO 100) unless that would need too long an exposure.
        exposure_time, gain = choose_gain_exposure(
            auto_exposure, auto_gain, base_gain=base_gain,
            max_gain=max_gain, max_exposure=max_exposure,
        )

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
        """Capture an HDR bracket by **shutter only** (constant gain).

        The rig is stationary while shooting the panorama, so every frame is a
        long exposure and there is no motion blur to trade against — bracketing
        by shutter keeps the ISO low (hence low noise). The gain stays at the
        calibrated (base) value for every frame; negative EV steps lengthen the
        exposure and positive EV steps shorten it. The [ISO 100 vs 200] choice
        and any exposure ceiling are handled in ``estimate_parameters()`` (see
        ``choose_gain_exposure``); here the exposures simply scale from the
        chosen base, however long that is.

        Filenames carry the EV offset (``..._<ev>.jpg``) so the Hugin reorder
        step can match them to the template's per-angle EV order.
        """
        path_split = os.path.splitext(path)
        AEB = int(AEB)
        stop_multiplier = 2 ** AEB_stops
        paths = []

        self._lock_3a()

        for step in range(-AEB, AEB + 1):
            # shutter-only bracket: gain constant, exposure scales per stop
            hdr_exposure_time = exposure_time * (stop_multiplier ** -step)

            self._set_manual(hdr_exposure_time, gain, awbgains)
            # set_controls applies to the NEXT frame — sync so the saved frame
            # really carries this exposure (also drains the startup auto frame)
            self._sync_to(hdr_exposure_time)
            hdr_path = f"{path_split[0]}_{step * AEB_stops}{path_split[1]}"
            self.camera.capture_file(hdr_path)
            paths.append(hdr_path)

        return paths
