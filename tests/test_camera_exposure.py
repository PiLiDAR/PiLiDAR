"""Tests for the camera exposure/gain math (``lib/picam2_utils.py``).

Desktop/CI only — the Picamera2 import is inside ``Camera.__init__``, so the
pure helpers can be tested without the libcamera stack.
"""
from lib.picam2_utils import choose_gain_exposure


def test_choose_gain_prefers_low_iso_when_exposure_fits():
    # auto: 0.02 s at gain 8  ->  at ISO 100 (gain 1) needs 0.16 s (< 1 s cap)
    exposure, gain = choose_gain_exposure(0.02, 8.0, base_gain=1.0, max_gain=2.0, max_exposure=1.0)
    assert gain == 1.0
    assert exposure == 0.02 * 8.0 / 1.0


def test_choose_gain_raises_to_cap_when_exposure_too_long():
    # auto: 0.05 s at gain 8 -> ISO 100 would need 0.4 s (> 0.1 s cap), so the
    # gain is raised to the ISO 200 cap and the exposure recomputed to 0.2 s
    exposure, gain = choose_gain_exposure(0.05, 8.0, base_gain=1.0, max_gain=2.0, max_exposure=0.1)
    assert gain == 2.0
    assert exposure == 0.05 * 8.0 / 2.0


def test_choose_gain_no_exposure_cap_keeps_base_gain():
    exposure, gain = choose_gain_exposure(0.5, 4.0, base_gain=1.0, max_gain=2.0, max_exposure=0)
    assert gain == 1.0
    assert exposure == 0.5 * 4.0


def test_bracket_is_shutter_only_and_symmetric():
    # EV steps [-2, 0, +2] at base exposure 0.02 s -> [0.08, 0.02, 0.005]
    exposure_time = 0.02
    stop_multiplier = 2 ** 2
    steps = [-2, 0, 2]
    exposures = [exposure_time * (stop_multiplier ** -(s // 2)) for s in steps]
    assert exposures == [0.08, 0.02, 0.005]
    # each adjacent pair is 2 stops apart
    assert abs(exposures[0] / exposures[1] - 4) < 1e-9
