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


# Default ISOs for the bracket (config: CAM.gain_low / CAM.gain_high).
GAIN_LOW = 1.0    # ISO 100
GAIN_HIGH = 2.0   # ISO 200


def illumination(exposure_time, gain):
    """Relative scene illumination captured by a frame: ``exposure_time × gain``.

    The auto-exposure solves for this quantity and the HDR bracket scales it.
    (Loosely an "EV"; strictly the captured light, with gain relative to ISO 100.)
    """
    return exposure_time * gain


def ev_transform(ev, gain_low=GAIN_LOW):
    """Turn an illumination value ``ev`` into the base ``(gain, exposure_time)``.

    Always uses the low ISO (``gain_low``); the exposure is simply ``ev/gain``.
    There is no exposure ceiling — a night scan may legitimately need seconds.
    """
    return gain_low, ev / gain_low


def bracket_exposures(ev, steps, gain_low=GAIN_LOW, gain_high=GAIN_HIGH):
    """Exposure set for an HDR bracket around ``ev``.

    ``steps`` are EV offsets (e.g. ``[-2, -1, 0, 1, 2]``; positive = brighter).
    The gain is **fixed per pass**: the low ISO for steps ≤ 0 and the high ISO
    for steps > 0 (so the positive/bright steps need less shutter). The target
    illumination is ``ev × 2**step``. Returns ``[(gain, exposure_time), ...]``
    in the given step order.
    """
    out = []
    for s in steps:
        gain = gain_high if s > 0 else gain_low
        target = ev * (2 ** s)
        out.append((gain, target / gain))
    return out


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
    def configure(self, still_dims, lores_dims=None):
        """Configure still + (optional) low-res streams and start the camera.

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
            lores={"size": tuple(lores_dims)} if lores_dims else None,
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

    def _sync_to(self, exposure_time, tolerance=0.05, attempts=12):
        """Capture one frame whose exposure (and gain) match the request.

        ``set_controls()`` applies to a *later* frame than the one already in
        flight, so ``capture_file()`` issued right after it often saves the
        previous exposure. To be sure, this grabs requests and checks their
        actual metadata, discarding any that do not match ``exposure_time``
        (within ``tolerance``) until one does — the saved frame then really
        carries the requested exposure, however short the bracket step is.
        """
        want_ns = _ns(exposure_time)
        slack = max(1_000_000, want_ns * tolerance)     # >= 1 ms or `tolerance`
        for _ in range(attempts):
            if abs(self.camera.capture_metadata()["ExposureTime"] - want_ns) <= slack:
                return True
        return False

    def _capture_at(self, path, exposure_time, gain, awbgains, retries=15):
        """Save a still and guarantee it carries the requested settings.

        ``set_controls()`` takes effect a frame or two *after* the frame already
        in flight, so a naive ``capture_file()`` saves the previous step's
        settings. This grabs requests and accepts only a frame whose reported
        ``ExposureTime`` and ``AnalogueGain`` really match the request — the
        tolerance is proportional (never an absolute millisecond floor, which
        would swallow the dark bracket steps). After ``retries`` it relaxes once
        rather than losing the shot.
        """
        want_ns = _ns(exposure_time)
        self._lock_3a()
        self._set_manual(exposure_time, gain, awbgains)

        def matches(meta, tol_e, tol_g):
            ok_e = abs(meta["ExposureTime"] - want_ns) <= want_ns * tol_e
            ok_g = abs(meta["AnalogueGain"] - gain) <= tol_g
            return ok_e and ok_g

        for _ in range(retries):
            request = self.camera.capture_request()
            meta = request.get_metadata()
            if matches(meta, 0.05, 0.02):        # strict
                request.save("main", path)
                request.release()
                return path
            request.release()                    # stale frame -> discard, retry

        # last resort: accept a looser exposure match (gain must still match)
        for _ in range(5):
            request = self.camera.capture_request()
            meta = request.get_metadata()
            if matches(meta, 0.5, 0.05):
                request.save("main", path)
                request.release()
                return path
            request.release()

        self.camera.capture_file(path)           # give up gracefully
        return path

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
        """Converge 3A at the calibration resolution, read the result, and lock it.

        Returns ``(illumination, gain, exposure_time, awbgains)``:
        ``illumination`` = auto exposure × auto gain (the scene "EV"), ``gain`` /
        ``exposure_time`` = the chosen manual setting for the base (EV 0) frame,
        and ``awbgains`` = ``[R, B]``.
        """
        calibration_dims = config.get("CAM", "calibration_dims")
        gain_low = config.get("CAM", "gain_low")

        # converge 3A at a small resolution (faster than full-res); the result
        # carries over unchanged to the full-resolution stills (same FOV)
        self.configure(still_dims=calibration_dims)
        self._settle()

        meta = self.camera.capture_metadata()
        auto_exposure = meta["ExposureTime"] / 1e9               # ns -> s
        auto_gain = meta["AnalogueGain"] * meta["DigitalGain"]   # total linear gain
        awbgains = list(meta["ColourGains"])                     # [R, B]

        self._lock_3a()

        ev = illumination(auto_exposure, auto_gain)
        gain, exposure_time = ev_transform(ev, gain_low=gain_low)

        return ev, gain, exposure_time, awbgains

    # -- capture -------------------------------------------------------------
    def take_photo(self, path, exposure_time, gain, awbgains):
        """Capture a single still at the given manual parameters.

        Assumes the camera is already configured (see ``configure()``).
        """
        return self._capture_at(path, exposure_time, gain, awbgains)

    def take_hdr(self, path, ev, AEB, AEB_stops, awbgains,
                 gain_low=GAIN_LOW, gain_high=GAIN_HIGH):
        """Capture an HDR bracket around ``ev`` (the measured illumination).

        The EV offsets are ``[-AEB*stops, …, 0, …, +AEB*stops]`` determined by
        ``AEB`` (number of stops each side) and ``AEB_stops`` (step size). Gain
        is fixed per pass — low ISO for the ≤0 (dark/neutral) frames and high ISO
        for the >0 (bright) frames — so ISO noise stays low. Filenames carry the
        EV offset (``..._<ev>.jpg``) so the Hugin reorder step can match them to
        the template's per-angle EV order.
        """
        path_split = os.path.splitext(path)
        AEB = int(AEB)
        steps = [s * AEB_stops for s in range(-AEB, AEB + 1)]
        bracket = bracket_exposures(ev, steps, gain_low=gain_low, gain_high=gain_high)
        paths = []

        self._lock_3a()

        for step, (gain, exposure_time) in zip(steps, bracket):
            hdr_path = f"{path_split[0]}_{step}{path_split[1]}"
            # capture-and-verify so the saved frame carries this exposure
            self._capture_at(hdr_path, exposure_time, gain, awbgains)
            paths.append(hdr_path)

        return paths
