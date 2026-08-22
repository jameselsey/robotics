"""Unit tests for hardware-independent differential-drive calculations."""

import math

import pytest

from drive_controller.kinematics import (
    Pose2D,
    encoder_delta_metres,
    integrate_differential_drive,
    metres_per_tick,
)


def test_calibrated_one_metre_encoder_run():
    distance = encoder_delta_metres(
        delta_ticks=1740,
        metres_per_encoder_tick=metres_per_tick(0.026, 230),
        distance_scale=0.81,
        inverted=False,
    )
    assert distance == pytest.approx(1.001, abs=0.002)


def test_left_encoder_inversion_produces_forward_distance():
    distance = encoder_delta_metres(
        delta_ticks=-1740,
        metres_per_encoder_tick=metres_per_tick(0.026, 230),
        distance_scale=0.81,
        inverted=True,
    )
    assert distance > 0.0


def test_equal_wheel_travel_moves_along_positive_x():
    pose, distance, heading_delta = integrate_differential_drive(
        Pose2D(0.0, 0.0, 0.0),
        left_distance_m=1.0,
        right_distance_m=1.0,
        wheel_base_m=0.19,
    )
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.theta == pytest.approx(0.0)
    assert distance == pytest.approx(1.0)
    assert heading_delta == pytest.approx(0.0)


def test_right_wheel_travel_turns_left():
    pose, _, heading_delta = integrate_differential_drive(
        Pose2D(0.0, 0.0, 0.0),
        left_distance_m=0.1,
        right_distance_m=0.2,
        wheel_base_m=0.2,
    )
    assert heading_delta == pytest.approx(0.5)
    assert pose.theta == pytest.approx(0.5)
    assert pose.y > 0.0


def test_heading_is_normalized_across_pi():
    pose, _, _ = integrate_differential_drive(
        Pose2D(0.0, 0.0, math.pi - 0.05),
        left_distance_m=0.0,
        right_distance_m=0.04,
        wheel_base_m=0.2,
    )
    assert -math.pi <= pose.theta <= math.pi
    assert pose.theta < 0.0


@pytest.mark.parametrize(
    ("radius", "counts"),
    [(0.0, 230), (-0.1, 230), (0.026, 0), (0.026, -1)],
)
def test_invalid_encoder_geometry_is_rejected(radius, counts):
    with pytest.raises(ValueError):
        metres_per_tick(radius, counts)
