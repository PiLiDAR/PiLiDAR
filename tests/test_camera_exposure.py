"""Tests for the camera exposure/gain math (``lib/picam2_utils.py``).

Desktop/CI only — the Picamera2 import is inside ``Camera.__init__``, so the
pure helpers can be tested without the libcamera stack.
"""
from lib.picam2_utils import bracket_exposures, ev_transform, illumination


def test_illumination_is_exposure_times_gain():
    assert illumination(0.02, 4.0) == 0.08


def test_ev_transform_low_iso_when_exposure_fits():
    # ev 0.16 -> at ISO 100 needs 0.16 s (< 1 s) -> ISO 100
    gain, exposure = ev_transform(0.16, max_exposure=1.0)
    assert gain == 1.0
    assert exposure == 0.16


def test_ev_transform_high_iso_when_exposure_too_long():
    # ev 4.0 -> ISO 100 would need 4 s (> 1 s) -> ISO 200, 2 s (no upper cap)
    gain, exposure = ev_transform(4.0, max_exposure=1.0)
    assert gain == 2.0
    assert exposure == 2.0


def test_ev_transform_no_cap_keeps_low_iso():
    gain, exposure = ev_transform(0.5, max_exposure=0)
    assert (gain, exposure) == (1.0, 0.5)


def test_bracket_half_exposure_dark_double_gain_bright():
    # base ev 0.2: dark steps use ISO 100 with proportionally shorter exposure,
    # bright steps use ISO 200 with the base exposure.
    result = bracket_exposures(0.2, [-1, 0, 1])
    assert result == [(1.0, 0.1), (1.0, 0.2), (2.0, 0.2)]


def test_bracket_gain_only_changes_for_positive_steps():
    result = bracket_exposures(0.2, [-2, -1, 0, 1, 2])
    gains = [g for g, _ in result]
    assert gains == [1.0, 1.0, 1.0, 2.0, 2.0]
    # illumination is monotonic in the step order (darker -> brighter)
    lums = [g * e for g, e in result]
    assert lums == sorted(lums)
