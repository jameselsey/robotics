"""Unit tests for semantic-room geometry and safe room-name resolution."""

import importlib
import sys
import types

import pytest


action_msgs = types.ModuleType("action_msgs")
action_msgs_msg = types.ModuleType("action_msgs.msg")
action_msgs_msg.GoalStatus = types.SimpleNamespace(
    STATUS_UNKNOWN=0,
    STATUS_ACCEPTED=1,
    STATUS_EXECUTING=2,
    STATUS_CANCELING=3,
    STATUS_SUCCEEDED=4,
    STATUS_CANCELED=5,
    STATUS_ABORTED=6,
)
action_msgs.msg = action_msgs_msg

geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.PoseStamped = object
geometry_msgs_msg.PoseWithCovarianceStamped = object
geometry_msgs.msg = geometry_msgs_msg

rclpy = types.ModuleType("rclpy")
rclpy.time = types.SimpleNamespace(Time=object)
rclpy_action = types.ModuleType("rclpy.action")
rclpy_action.ActionClient = object
rclpy_duration = types.ModuleType("rclpy.duration")
rclpy_duration.Duration = object

strands = types.ModuleType("strands")
strands.tool = lambda function: function

tf2_ros = types.ModuleType("tf2_ros")
tf2_ros.Buffer = object
tf2_ros.TransformException = Exception
tf2_ros.TransformListener = object

sys.modules.setdefault("action_msgs", action_msgs)
sys.modules.setdefault("action_msgs.msg", action_msgs_msg)
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)
sys.modules.setdefault("rclpy", rclpy)
sys.modules.setdefault("rclpy.action", rclpy_action)
sys.modules.setdefault("rclpy.duration", rclpy_duration)
sys.modules.setdefault("strands", strands)
sys.modules.setdefault("tf2_ros", tf2_ros)

# Other test modules may have installed a smaller geometry_msgs stub first.
geometry_msgs_msg = sys.modules["geometry_msgs.msg"]
if not hasattr(geometry_msgs_msg, "PoseStamped"):
    geometry_msgs_msg.PoseStamped = object
if not hasattr(geometry_msgs_msg, "PoseWithCovarianceStamped"):
    geometry_msgs_msg.PoseWithCovarianceStamped = object
sys.modules["geometry_msgs"].msg = geometry_msgs_msg

semantic = importlib.import_module("senses.semantic_map_tools")


def test_spoken_number_resolves_exact_room():
    key, ambiguous = semantic._resolve_room_name(
        ["bed 1", "bed 2", "hallway"],
        "Bed one",
    )
    assert key == "bed 1"
    assert ambiguous == []


def test_ambiguous_room_name_is_not_guessed():
    key, ambiguous = semantic._resolve_room_name(["bed 1", "bed 2"], "bed")
    assert key is None
    assert ambiguous == ["bed 1", "bed 2"]


def test_unique_partial_room_name_is_supported():
    key, ambiguous = semantic._resolve_room_name(
        ["bed 1", "hallway", "lounge"],
        "hall",
    )
    assert key == "hallway"
    assert ambiguous == []


@pytest.mark.parametrize(
    ("point", "expected"),
    [((1.0, 1.0), True), ((3.0, 1.0), False), ((-1.0, -1.0), False)],
)
def test_point_in_polygon(point, expected):
    polygon = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    assert semantic._point_in_polygon(*point, polygon) is expected


def test_polygon_centroid():
    polygon = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]
    assert semantic._polygon_centroid(polygon) == pytest.approx((1.0, 1.0))


def test_reviewed_navigation_pose_takes_precedence_over_centroid():
    controller = semantic.SemanticMapController.__new__(semantic.SemanticMapController)
    goal = controller._room_goal(
        "bed 1",
        {
            "navigate_pose": {"x": 2.4, "y": -0.1, "yaw": 0.5},
            "polygon": [[0, 0], [1, 0], [1, 1]],
        },
    )
    assert goal[:3] == pytest.approx((2.4, -0.1, 0.5))
    assert goal[3] == "navigate_pose"


def test_navigation_rejects_missing_localization_estimate():
    error = semantic._localization_confidence_error(None)
    assert "not published" in error


def test_navigation_accepts_confident_localization_estimate():
    covariance = [0.0] * 36
    covariance[0] = 0.10
    covariance[7] = 0.12
    covariance[35] = 0.20
    assert semantic._localization_confidence_error(covariance) is None


@pytest.mark.parametrize("index", [0, 7, 35])
def test_navigation_rejects_uncertain_localization_axis(index):
    covariance = [0.0] * 36
    covariance[index] = 0.30
    error = semantic._localization_confidence_error(covariance)
    assert "too uncertain" in error
