"""Tests for movement_mcp_server MCP tools.

Covers all 8 correctness properties from design.md using Hypothesis,
plus unit tests for boundary values and specific examples.
"""

import math
import sys
import types
import unittest
from dataclasses import dataclass
from typing import Optional
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Stub out ROS2 / rclpy before importing the module under test so tests run
# without a live ROS2 graph.
# ---------------------------------------------------------------------------

# Build a minimal rclpy stub
rclpy_stub = types.ModuleType("rclpy")
rclpy_stub.init = MagicMock()
rclpy_stub.shutdown = MagicMock()
rclpy_stub.spin = MagicMock()

rclpy_node_stub = types.ModuleType("rclpy.node")


class _FakeNode:
    def __init__(self, *args, **kwargs):
        pass

    def create_publisher(self, *args, **kwargs):
        return MagicMock()

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def get_logger(self):
        return MagicMock()


rclpy_node_stub.Node = _FakeNode

geometry_msgs_stub = types.ModuleType("geometry_msgs")
geometry_msgs_msg_stub = types.ModuleType("geometry_msgs.msg")


class _FakeTwist:
    def __init__(self):
        self.linear = MagicMock()
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular = MagicMock()
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0


geometry_msgs_msg_stub.Twist = _FakeTwist

nav_msgs_stub = types.ModuleType("nav_msgs")
nav_msgs_msg_stub = types.ModuleType("nav_msgs.msg")
nav_msgs_msg_stub.Odometry = MagicMock()

mcp_stub = types.ModuleType("mcp")
mcp_server_stub = types.ModuleType("mcp.server")
mcp_server_fastmcp_stub = types.ModuleType("mcp.server.fastmcp")


class _FakeFastMCP:
    def __init__(self, *args, **kwargs):
        pass

    def tool(self):
        def decorator(fn):
            return fn
        return decorator

    def run(self, *args, **kwargs):
        pass


mcp_server_fastmcp_stub.FastMCP = _FakeFastMCP

sys.modules.setdefault("rclpy", rclpy_stub)
sys.modules.setdefault("rclpy.node", rclpy_node_stub)
sys.modules.setdefault("geometry_msgs", geometry_msgs_stub)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg_stub)
sys.modules.setdefault("nav_msgs", nav_msgs_stub)
sys.modules.setdefault("nav_msgs.msg", nav_msgs_msg_stub)
sys.modules.setdefault("mcp", mcp_stub)
sys.modules.setdefault("mcp.server", mcp_server_stub)
sys.modules.setdefault("mcp.server.fastmcp", mcp_server_fastmcp_stub)

# Now import the module under test
import importlib  # noqa: E402
import senses.movement_mcp_server as mms  # noqa: E402

from hypothesis import given, settings  # noqa: E402
import hypothesis.strategies as st  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node() -> mms.MovementNode:
    """Create a MovementNode with a mocked publisher."""
    node = mms.MovementNode.__new__(mms.MovementNode)
    _FakeNode.__init__(node)
    node._cmd_vel_pub = MagicMock()
    node._odom_sub = MagicMock()
    node._odom_cache = None
    import threading
    node._stop_event = threading.Event()
    return node


def _setup_node():
    """Inject a fresh mock node into the module and return it."""
    node = _make_node()
    mms._node = node
    return node


# ---------------------------------------------------------------------------
# Unit tests — validation helpers
# ---------------------------------------------------------------------------

class TestValidateLinear(unittest.TestCase):

    def test_zero_speed_raises(self):
        with self.assertRaises(ValueError):
            mms._validate_linear(0.0, 1.0)

    def test_negative_speed_raises(self):
        with self.assertRaises(ValueError):
            mms._validate_linear(-0.1, 1.0)

    def test_zero_duration_raises(self):
        with self.assertRaises(ValueError):
            mms._validate_linear(0.3, 0.0)

    def test_negative_duration_raises(self):
        with self.assertRaises(ValueError):
            mms._validate_linear(0.3, -1.0)

    def test_valid_speed_no_clamp(self):
        speed, duration, warnings = mms._validate_linear(0.5, 5.0)
        self.assertEqual(speed, 0.5)
        self.assertEqual(duration, 5.0)
        self.assertEqual(warnings, [])

    def test_speed_clamped(self):
        speed, duration, warnings = mms._validate_linear(1.0, 5.0)
        self.assertEqual(speed, mms.MAX_LINEAR_SPEED_MS)
        self.assertTrue(any("clamped" in w for w in warnings))

    def test_duration_clamped(self):
        speed, duration, warnings = mms._validate_linear(0.3, 15.0)
        self.assertEqual(duration, mms.MAX_DURATION_S)
        self.assertTrue(any("clamped" in w for w in warnings))

    def test_boundary_speed_not_clamped(self):
        speed, _, warnings = mms._validate_linear(0.5, 1.0)
        self.assertEqual(speed, 0.5)
        self.assertEqual(warnings, [])

    def test_boundary_duration_not_clamped(self):
        _, duration, warnings = mms._validate_linear(0.3, 10.0)
        self.assertEqual(duration, 10.0)
        self.assertEqual(warnings, [])


class TestValidateAngular(unittest.TestCase):

    def test_zero_speed_raises(self):
        with self.assertRaises(ValueError):
            mms._validate_angular(0.0, 1.0)

    def test_negative_speed_raises(self):
        with self.assertRaises(ValueError):
            mms._validate_angular(-1.0, 1.0)

    def test_valid_speed_no_clamp(self):
        speed, duration, warnings = mms._validate_angular(1.0, 2.0)
        self.assertEqual(speed, 1.0)
        self.assertEqual(warnings, [])

    def test_speed_clamped(self):
        speed, _, warnings = mms._validate_angular(3.0, 1.0)
        self.assertEqual(speed, mms.MAX_ANGULAR_SPEED_RADS)
        self.assertTrue(any("clamped" in w for w in warnings))

    def test_boundary_speed_not_clamped(self):
        speed, _, warnings = mms._validate_angular(2.0, 1.0)
        self.assertEqual(speed, 2.0)
        self.assertEqual(warnings, [])


# ---------------------------------------------------------------------------
# Unit tests — MCP tools
# ---------------------------------------------------------------------------

class TestMoveForward(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    def test_valid_call_publishes_positive_linear_x(self):
        published = []
        self.node._publish_twist = lambda lx, az: published.append((lx, az))
        self.node._timed_move = lambda lx, az, dur: published.append((lx, az))
        result = mms.move_forward(0.3, 1.0)
        self.assertIn("Moving forward", result)
        self.assertGreater(published[0][0], 0.0)
        self.assertEqual(published[0][1], 0.0)

    def test_invalid_speed_returns_error(self):
        result = mms.move_forward(0.0, 1.0)
        self.assertTrue(result.startswith("ERROR"))

    def test_invalid_duration_returns_error(self):
        result = mms.move_forward(0.3, 0.0)
        self.assertTrue(result.startswith("ERROR"))

    def test_clamped_speed_includes_warning(self):
        published = []
        self.node._timed_move = lambda lx, az, dur: published.append((lx, az))
        result = mms.move_forward(1.0, 1.0)
        self.assertIn("WARNING", result)
        self.assertEqual(published[0][0], mms.MAX_LINEAR_SPEED_MS)


class TestMoveBackward(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    def test_valid_call_publishes_negative_linear_x(self):
        published = []
        self.node._timed_move = lambda lx, az, dur: published.append((lx, az))
        result = mms.move_backward(0.3, 1.0)
        self.assertIn("Moving backward", result)
        self.assertLess(published[0][0], 0.0)
        self.assertEqual(published[0][1], 0.0)

    def test_invalid_speed_returns_error(self):
        result = mms.move_backward(-0.1, 1.0)
        self.assertTrue(result.startswith("ERROR"))


class TestTurnLeft(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    def test_valid_call_publishes_positive_angular_z(self):
        published = []
        self.node._timed_move = lambda lx, az, dur: published.append((lx, az))
        result = mms.turn_left(1.0, 1.0)
        self.assertIn("Turning left", result)
        self.assertEqual(published[0][0], 0.0)
        self.assertGreater(published[0][1], 0.0)

    def test_invalid_speed_returns_error(self):
        result = mms.turn_left(0.0, 1.0)
        self.assertTrue(result.startswith("ERROR"))


class TestTurnRight(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    def test_valid_call_publishes_negative_angular_z(self):
        published = []
        self.node._timed_move = lambda lx, az, dur: published.append((lx, az))
        result = mms.turn_right(1.0, 1.0)
        self.assertIn("Turning right", result)
        self.assertEqual(published[0][0], 0.0)
        self.assertLess(published[0][1], 0.0)

    def test_invalid_speed_returns_error(self):
        result = mms.turn_right(-1.0, 1.0)
        self.assertTrue(result.startswith("ERROR"))


class TestStop(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    def test_stop_publishes_zero_twist(self):
        published = []
        self.node._publish_twist = lambda lx, az: published.append((lx, az))
        result = mms.stop()
        self.assertEqual(result, "Robot stopped")
        self.assertEqual(published[-1], (0.0, 0.0))

    def test_stop_sets_stop_event(self):
        self.node._publish_twist = MagicMock()
        mms.stop()
        self.assertTrue(self.node._stop_event.is_set())


class TestGetOdometry(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    def test_no_odom_returns_not_available(self):
        self.node._odom_cache = None
        result = mms.get_odometry()
        self.assertIn("not yet available", result)

    def test_odom_returns_formatted_string(self):
        self.node._odom_cache = mms.OdomSnapshot(
            x=1.0, y=2.0, theta_deg=90.0,
            linear_vel=0.1, angular_vel=0.2, stamp=0.0
        )
        result = mms.get_odometry()
        self.assertIn("x=1.000", result)
        self.assertIn("y=2.000", result)
        self.assertIn("theta=90.0", result)
        self.assertIn("linear_vel=0.100", result)
        self.assertIn("angular_vel=0.200", result)


# ---------------------------------------------------------------------------
# Property-based tests
# ---------------------------------------------------------------------------

class TestProperties(unittest.TestCase):

    def setUp(self):
        self.node = _setup_node()

    # Feature: robot-movement-mcp-tools, Property 1: Valid linear speed is published unchanged
    @given(
        speed_ms=st.floats(min_value=0.001, max_value=0.5, allow_nan=False),
        duration_s=st.floats(min_value=0.001, max_value=10.0, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_valid_speed_published_unchanged(self, speed_ms, duration_s):
        """Validates: Requirements 2.2, 3.2"""
        node = _setup_node()
        published = []
        node._timed_move = lambda lx, az, dur: published.append((lx, az))

        mms.move_forward(speed_ms, duration_s)
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(abs(published[0][0]), speed_ms, places=9)
        self.assertEqual(published[0][1], 0.0)

        published.clear()
        mms.move_backward(speed_ms, duration_s)
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(abs(published[0][0]), speed_ms, places=9)
        self.assertEqual(published[0][1], 0.0)

    # Feature: robot-movement-mcp-tools, Property 2: Linear speed clamping
    @given(
        speed_ms=st.floats(min_value=0.501, max_value=100.0, allow_nan=False),
        duration_s=st.floats(min_value=0.001, max_value=10.0, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_linear_speed_clamped(self, speed_ms, duration_s):
        """Validates: Requirements 2.6, 3.5, 9.1"""
        node = _setup_node()
        published = []
        node._timed_move = lambda lx, az, dur: published.append((lx, az))

        result = mms.move_forward(speed_ms, duration_s)
        self.assertAlmostEqual(abs(published[0][0]), mms.MAX_LINEAR_SPEED_MS, places=9)
        self.assertTrue("WARNING" in result or "clamped" in result)

    # Feature: robot-movement-mcp-tools, Property 3: Angular speed clamping
    @given(
        angular_speed_rads=st.floats(min_value=2.001, max_value=100.0, allow_nan=False),
        duration_s=st.floats(min_value=0.001, max_value=10.0, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_angular_speed_clamped(self, angular_speed_rads, duration_s):
        """Validates: Requirements 4.4 (implicit), 5.4 (implicit), 9.4, 9.5"""
        node = _setup_node()
        published = []
        node._timed_move = lambda lx, az, dur: published.append((lx, az))

        result = mms.turn_left(angular_speed_rads, duration_s)
        self.assertAlmostEqual(abs(published[0][1]), mms.MAX_ANGULAR_SPEED_RADS, places=9)
        self.assertTrue("WARNING" in result or "clamped" in result)

    # Feature: robot-movement-mcp-tools, Property 4: Duration clamping
    @given(
        speed_ms=st.floats(min_value=0.001, max_value=0.5, allow_nan=False),
        duration_s=st.floats(min_value=10.001, max_value=1000.0, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_duration_clamped(self, speed_ms, duration_s):
        """Validates: Requirements 9.2, 9.3"""
        node = _setup_node()
        durations = []
        node._timed_move = lambda lx, az, dur: durations.append(dur)

        result = mms.move_forward(speed_ms, duration_s)
        self.assertAlmostEqual(durations[0], mms.MAX_DURATION_S, places=9)
        self.assertTrue("WARNING" in result or "clamped" in result)

    # Feature: robot-movement-mcp-tools, Property 5: Invalid parameters produce error strings
    @given(
        speed_ms=st.floats(max_value=0.0, allow_nan=False),
        duration_s=st.floats(min_value=0.001, max_value=10.0, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_invalid_speed_returns_error(self, speed_ms, duration_s):
        """Validates: Requirements 2.4, 2.5, 3.4, 4.4, 4.5, 5.4, 5.5"""
        node = _setup_node()
        published = []
        node._timed_move = lambda lx, az, dur: published.append((lx, az))

        result = mms.move_forward(speed_ms, duration_s)
        self.assertTrue(result.startswith("ERROR"))
        self.assertEqual(len(published), 0, "No Twist should be published on error")

    # Feature: robot-movement-mcp-tools, Property 6: Stop publishes zero velocity
    def test_stop_publishes_zero_twist(self):
        """Validates: Requirements 6.2"""
        node = _setup_node()
        published = []
        node._publish_twist = lambda lx, az: published.append((lx, az))

        mms.stop()
        self.assertTrue(len(published) >= 1)
        self.assertEqual(published[-1], (0.0, 0.0))

    # Feature: robot-movement-mcp-tools, Property 7: Turn direction sign convention
    @given(
        angular_speed_rads=st.floats(min_value=0.001, max_value=2.0, allow_nan=False),
        duration_s=st.floats(min_value=0.001, max_value=10.0, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_turn_direction_signs(self, angular_speed_rads, duration_s):
        """Validates: Requirements 4.2, 5.2"""
        node = _setup_node()
        left_published = []
        node._timed_move = lambda lx, az, dur: left_published.append((lx, az))
        mms.turn_left(angular_speed_rads, duration_s)
        self.assertGreater(left_published[0][1], 0.0)
        self.assertAlmostEqual(left_published[0][1], angular_speed_rads, places=9)

        right_published = []
        node._timed_move = lambda lx, az, dur: right_published.append((lx, az))
        mms.turn_right(angular_speed_rads, duration_s)
        self.assertLess(right_published[0][1], 0.0)
        self.assertAlmostEqual(abs(right_published[0][1]), angular_speed_rads, places=9)

    # Feature: robot-movement-mcp-tools, Property 8: Odometry round-trip
    @given(
        x=st.floats(allow_nan=False, allow_infinity=False, min_value=-1e6, max_value=1e6),
        y=st.floats(allow_nan=False, allow_infinity=False, min_value=-1e6, max_value=1e6),
        theta=st.floats(min_value=-math.pi, max_value=math.pi, allow_nan=False),
    )
    @settings(max_examples=100)
    def test_odometry_round_trip(self, x, y, theta):
        """Validates: Requirements 7.2, 7.4"""
        node = _setup_node()
        theta_deg = math.degrees(theta)
        node._odom_cache = mms.OdomSnapshot(
            x=x, y=y, theta_deg=theta_deg,
            linear_vel=0.0, angular_vel=0.0, stamp=0.0
        )
        result = mms.get_odometry()
        self.assertIn(f"x={x:.3f}", result)
        self.assertIn(f"y={y:.3f}", result)
        self.assertIn(f"theta={theta_deg:.1f}", result)


if __name__ == "__main__":
    unittest.main()
