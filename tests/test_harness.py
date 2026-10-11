"""Mock hardware harness tests + HDR bracket filename ordering."""
import os

from tests.mock_hardware import MockCamera, MockStepper


def test_mock_stepper_move_to_angle():
    stepper = MockStepper(step_angle=1.8, microsteps=16, gear_ratio=1.0)
    stepper.move_to_angle(90.0)
    assert abs(stepper.get_current_angle() - 90.0) < 0.2


def test_mock_stepper_wraps_360():
    stepper = MockStepper()
    stepper.move_to_angle(350.0)
    stepper.move_to_angle(10.0)
    # moving "forward" past 360 should wrap, not go backwards
    assert abs(stepper.get_current_angle() - 10.0) < 1.0


def test_mock_camera_hdr_order(tmp_path):
    """HDR brackets emit EV steps from -AEB..+AEB (capture order)."""
    cam = MockCamera()
    paths = cam.take_hdr(str(tmp_path / "image_0.jpg"), AEB=2, AEB_stops=2,
                         exposure_time=0.001, gain=1.0, awbgains=[1, 1])
    names = [os.path.basename(p) for p in paths]
    assert names == [
        "image_0_-4.jpg", "image_0_-2.jpg", "image_0_0.jpg",
        "image_0_2.jpg", "image_0_4.jpg",
    ]


def test_mock_camera_photo_recorded(tmp_path):
    cam = MockCamera()
    path = str(tmp_path / "shot.jpg")
    assert cam.take_photo(path, 0.001, 1.0, [1, 1]) == path
    assert cam.captures == [path]
