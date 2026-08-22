"""Fast unit tests for current movement validation and odometry helpers."""

import importlib
import math
import sys
import types

import pytest


class _Vector:
    x = 0.0
    y = 0.0
    z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vector()
        self.angular = _Vector()


geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.Twist = _Twist
geometry_msgs.msg = geometry_msgs_msg

nav_msgs = types.ModuleType("nav_msgs")
nav_msgs_msg = types.ModuleType("nav_msgs.msg")
nav_msgs_msg.Odometry = object
nav_msgs.msg = nav_msgs_msg

strands = types.ModuleType("strands")
strands.tool = lambda function: function

sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)
sys.modules.setdefault("nav_msgs", nav_msgs)
sys.modules.setdefault("nav_msgs.msg", nav_msgs_msg)
sys.modules.setdefault("strands", strands)

movement_tools = importlib.import_module("senses.movement_tools")


@pytest.mark.parametrize(
    ("speed", "duration"),
    [(0.0, 1.0), (-0.1, 1.0), (0.1, 0.0), (0.1, -1.0)],
)
def test_linear_validation_rejects_non_positive_values(speed, duration):
    with pytest.raises(ValueError):
        movement_tools._validate_linear(speed, duration)


def test_linear_validation_clamps_to_safety_limits():
    speed, duration, warnings = movement_tools._validate_linear(5.0, 100.0)
    assert speed == movement_tools.MAX_LINEAR_SPEED_MS
    assert duration == movement_tools.MAX_DURATION_S
    assert len(warnings) == 2


def test_distance_validation_clamps_distance_and_speed():
    distance, speed, warnings = movement_tools._validate_distance(10.0, 5.0)
    assert distance == movement_tools.MAX_DISTANCE_M
    assert speed == movement_tools.MAX_LINEAR_SPEED_MS
    assert len(warnings) == 2


@pytest.mark.parametrize(
    ("current", "previous", "expected"),
    [(10.0, 350.0, 20.0), (350.0, 10.0, -20.0), (45.0, 40.0, 5.0)],
)
def test_angle_delta_wraps_at_360_degrees(current, previous, expected):
    assert movement_tools._angle_delta_degrees(current, previous) == pytest.approx(expected)


def test_odometry_callback_extracts_planar_pose_and_velocity():
    orientation = types.SimpleNamespace(
        z=math.sin(math.radians(45.0)),
        w=math.cos(math.radians(45.0)),
    )
    message = types.SimpleNamespace(
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.2, y=-0.4),
                orientation=orientation,
            )
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.2),
                angular=types.SimpleNamespace(z=-0.1),
            )
        ),
    )
    controller = movement_tools.MovementController.__new__(movement_tools.MovementController)
    controller._odom_cache = None
    controller._odom_callback(message)

    assert controller._odom_cache.x == pytest.approx(1.2)
    assert controller._odom_cache.y == pytest.approx(-0.4)
    assert controller._odom_cache.theta_deg == pytest.approx(90.0)
    assert controller._odom_cache.linear_vel == pytest.approx(0.2)
    assert controller._odom_cache.angular_vel == pytest.approx(-0.1)
