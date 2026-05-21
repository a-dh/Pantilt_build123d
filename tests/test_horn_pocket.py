import pytest
from build123d.geometry import Axis

from pantilt_build123d.sg9_servo_horn import SG9ServoHorn, SG9ServoHornPocket


def assert_contains(horn, pocket):
    """Assert the pocket fully contains the horn without interference."""
    difference = horn - pocket
    assert difference.volume == pytest.approx(0, abs=1e-6), "Horn is not fully contained in the pocket"
    intersection = horn & pocket
    assert intersection.volume == pytest.approx(horn.volume, abs=1e-6)


def test_horn_pocket_contains_canonical_horn():
    horn = SG9ServoHorn()
    pocket = SG9ServoHornPocket(horn, clearance=0.2)

    assert_contains(horn, pocket)


def test_horn_pocket_contains_diagonal_horn():
    horn = SG9ServoHorn().rotate(Axis.Z, 45)
    pocket = SG9ServoHornPocket(horn, clearance=0.2)

    assert_contains(horn, pocket)
