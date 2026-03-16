"""MCP server exposing robot movement tools via ROS2 /cmd_vel and /odom."""

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mcp.server.fastmcp import FastMCP

# ---------------------------------------------------------------------------
# Safety constants
# ---------------------------------------------------------------------------
MAX_LINEAR_SPEED_MS = 0.5    # m/s
MAX_ANGULAR_SPEED_RADS = 2.0  # rad/s
MAX_DURATION_S = 10.0         # seconds

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------
@dataclass
class OdomSnapshot:
    x: float
    y: float
    theta_deg: float
    linear_vel: float
    angular_vel: float
    stamp: float


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class MovementNode(Node):
    """ROS2 node that owns the /cmd_vel publisher and /odom subscriber."""

    def __init__(self):
        super().__init__("movement_mcp_server")

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            10,
        )

        self._odom_cache: Optional[OdomSnapshot] = None
        self._stop_event: threading.Event = threading.Event()

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache the latest odometry message as an OdomSnapshot."""
        pos = msg.pose.pose.position
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta_deg = math.degrees(2 * math.atan2(qz, qw))

        self._odom_cache = OdomSnapshot(
            x=pos.x,
            y=pos.y,
            theta_deg=theta_deg,
            linear_vel=msg.twist.twist.linear.x,
            angular_vel=msg.twist.twist.angular.z,
            stamp=time.time(),
        )

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        """Publish a single Twist message to /cmd_vel."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    def _timed_move(
        self, linear_x: float, angular_z: float, duration_s: float
    ) -> None:
        """Publish a move command, wait for duration or stop event, then halt."""
        self._stop_event.clear()
        self._publish_twist(linear_x, angular_z)
        self._stop_event.wait(timeout=duration_s)
        self._publish_twist(0.0, 0.0)


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------
def _validate_linear(
    speed_ms: float, duration_s: float
) -> tuple[float, float, list[str]]:
    """Validate and clamp linear movement parameters.

    Returns (clamped_speed, clamped_duration, warnings).
    Raises ValueError for non-positive inputs.
    """
    if speed_ms <= 0.0:
        raise ValueError("speed_ms must be > 0.0")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be > 0.0")

    warnings: list[str] = []

    if speed_ms > MAX_LINEAR_SPEED_MS:
        warnings.append(
            f"speed clamped from {speed_ms:.3f} to {MAX_LINEAR_SPEED_MS} m/s"
        )
        speed_ms = MAX_LINEAR_SPEED_MS

    if duration_s > MAX_DURATION_S:
        warnings.append(
            f"duration clamped from {duration_s:.1f} to {MAX_DURATION_S} s"
        )
        duration_s = MAX_DURATION_S

    return speed_ms, duration_s, warnings


def _validate_angular(
    angular_speed_rads: float, duration_s: float
) -> tuple[float, float, list[str]]:
    """Validate and clamp angular movement parameters.

    Returns (clamped_speed, clamped_duration, warnings).
    Raises ValueError for non-positive inputs.
    """
    if angular_speed_rads <= 0.0:
        raise ValueError("angular_speed_rads must be > 0.0")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be > 0.0")

    warnings: list[str] = []

    if angular_speed_rads > MAX_ANGULAR_SPEED_RADS:
        warnings.append(
            f"angular speed clamped from {angular_speed_rads:.3f} to {MAX_ANGULAR_SPEED_RADS} rad/s"
        )
        angular_speed_rads = MAX_ANGULAR_SPEED_RADS

    if duration_s > MAX_DURATION_S:
        warnings.append(
            f"duration clamped from {duration_s:.1f} to {MAX_DURATION_S} s"
        )
        duration_s = MAX_DURATION_S

    return angular_speed_rads, duration_s, warnings


# ---------------------------------------------------------------------------
# MCP server instance (module-level so tool decorators can reference it)
# ---------------------------------------------------------------------------
mcp = FastMCP("robot-movement")


@mcp.tool()
def move_forward(speed_ms: float, duration_s: float) -> str:
    """Move the robot forward at the given speed for the given duration."""
    try:
        speed, duration, warnings = _validate_linear(speed_ms, duration_s)
    except ValueError as e:
        return f"ERROR: {e}"
    _node._timed_move(speed, 0.0, duration)
    msg = f"Moving forward at {speed} m/s for {duration} s"
    if warnings:
        msg += " [WARNING: " + "; ".join(warnings) + "]"
    return msg


@mcp.tool()
def move_backward(speed_ms: float, duration_s: float) -> str:
    """Move the robot backward at the given speed for the given duration."""
    try:
        speed, duration, warnings = _validate_linear(speed_ms, duration_s)
    except ValueError as e:
        return f"ERROR: {e}"
    _node._timed_move(-speed, 0.0, duration)
    msg = f"Moving backward at {speed} m/s for {duration} s"
    if warnings:
        msg += " [WARNING: " + "; ".join(warnings) + "]"
    return msg


@mcp.tool()
def turn_left(angular_speed_rads: float, duration_s: float) -> str:
    """Turn the robot left (counter-clockwise) at the given angular speed for the given duration."""
    try:
        speed, duration, warnings = _validate_angular(angular_speed_rads, duration_s)
    except ValueError as e:
        return f"ERROR: {e}"
    _node._timed_move(0.0, speed, duration)
    msg = f"Turning left at {speed} rad/s for {duration} s"
    if warnings:
        msg += " [WARNING: " + "; ".join(warnings) + "]"
    return msg


@mcp.tool()
def turn_right(angular_speed_rads: float, duration_s: float) -> str:
    """Turn the robot right (clockwise) at the given angular speed for the given duration."""
    try:
        speed, duration, warnings = _validate_angular(angular_speed_rads, duration_s)
    except ValueError as e:
        return f"ERROR: {e}"
    _node._timed_move(0.0, -speed, duration)
    msg = f"Turning right at {speed} rad/s for {duration} s"
    if warnings:
        msg += " [WARNING: " + "; ".join(warnings) + "]"
    return msg


@mcp.tool()
def stop() -> str:
    """Immediately stop all robot motion."""
    _node._stop_event.set()
    _node._publish_twist(0.0, 0.0)
    return "Robot stopped"


@mcp.tool()
def get_odometry() -> str:
    """Return the robot's current position and heading from odometry."""
    snap = _node._odom_cache
    if snap is None:
        return "Odometry not yet available"
    return (
        f"x={snap.x:.3f} m, y={snap.y:.3f} m, theta={snap.theta_deg:.1f} deg, "
        f"linear_vel={snap.linear_vel:.3f} m/s, angular_vel={snap.angular_vel:.3f} rad/s"
    )


# Module-level reference to the node, set in main()
_node: Optional[MovementNode] = None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    """Initialise ROS2, start the MCP server over stdio."""
    global _node

    rclpy.init(args=args)
    _node = MovementNode()

    # Spin the ROS2 node in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()

    try:
        mcp.run(transport="stdio")
    finally:
        # Publish zero-velocity before shutting down
        stop_twist = Twist()
        _node._cmd_vel_pub.publish(stop_twist)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
