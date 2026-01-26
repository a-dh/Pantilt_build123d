import pytest
from build123d import Face, Shape

from pantilt_build123d.sg9_servo import SG9Servo_Assembly


@pytest.mark.parametrize(
    "left,right,expect_left_none,expect_right_none",
    [
        (True,  False, False, True),  # left only
        (False, True,  True,  False), # right only
        (True,  True,  False, False), # both
        (False,  False, True, True),  # neither
    ],
)
def test_mounts(
    left: bool,
    right: bool,
    expect_left_none: bool,
    expect_right_none: bool,
):
    servo = SG9Servo_Assembly(left_mount=left, right_mount=right)
    mounts = servo.mounts()

    assert (mounts["left_mount"] is None) == expect_left_none
    assert (mounts["right_mount"] is None) == expect_right_none

def test_direct_references():
    servo = SG9Servo_Assembly()
    # just test that these attributes exist and are of the expected type
    assert isinstance(servo.final_shaft, Shape)
    assert isinstance(servo.final_shaft_top_face, Face)
    assert isinstance(servo.final_shaft_radius, float)
