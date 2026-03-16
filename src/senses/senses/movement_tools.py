"""Movement tools for the Strands agent, bound to a ROS2 publisher."""

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from strands import tool

MAX_LINEAR_SPEED_MS = 0.5
MAX_ANGULAR_SPEED_RADS = 2.0
MAX_DURATION_S = 10.0


@dataclass
class OdomSnapshot:
    x: float
    y: float
    theta_deg: float
    linear_vel: float
    angular_vel: float


def _validate_linear(speed_ms: float, duration_s: float):
    if speed_ms <= 0.0:
        raise ValueError("speed_ms must be > 0.0")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be > 0.0")
    warnings = []
    if speed_ms > MAX_LINEAR_SPEED_MS:
        warnings.append(f"speed clamped from {speed_ms:.3f} to {MAX_LINEAR_SPEED_MS} m/s")
        speed_ms = MAX_LINEAR_SPEED_MS
    if duration_s > MAX_DURATION_S:
        warnings.append(f"duration clamped from {duration_s:.1f} to {MAX_DURATION_S} s")
        duration_s = MAX_DURATION_S
    return speed_ms, duration_s, warnings


def _validate_angular(angular_speed_rads: float, duration_s: float):
    if angular_speed_rads <= 0.0:
        raise ValueError("angular_speed_rads must be > 0.0")
    if duration_s <= 0.0:
        raise ValueError("duration_s must be > 0.0")
    warnings = []
    if angular_speed_rads > MAX_ANGULAR_SPEED_RADS:
        warnings.append(f"angular speed clamped from {angular_speed_rads:.3f} to {MAX_ANGULAR_SPEED_RADS} rad/s")
        angular_speed_rads = MAX_ANGULAR_SPEED_RADS
    if duration_s > MAX_DURATION_S:
        warnings.append(f"duration clamped from {duration_s:.1f} to {MAX_DURATION_S} s")
        duration_s = MAX_DURATION_S
    return angular_speed_rads, duration_s, warnings


class MovementController:
    """Owns the /cmd_vel publisher and /odom subscriber state."""

    def __init__(self, ros_node):
        self._pub = ros_node.create_publisher(Twist, "/cmd_vel", 10)
        ros_node.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self._odom_cache: Optional[OdomSnapshot] = None
        self._stop_event = threading.Event()

    def _odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._odom_cache = OdomSnapshot(
            x=pos.x, y=pos.y,
            theta_deg=math.degrees(2 * math.atan2(qz, qw)),
            linear_vel=msg.twist.twist.linear.x,
            angular_vel=msg.twist.twist.angular.z,
        )

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._pub.publish(msg)

    def timed_move(self, linear_x: float, angular_z: float, duration_s: float) -> None:
        self._stop_event.clear()
        self.publish_twist(linear_x, angular_z)
        self._stop_event.wait(timeout=duration_s)
        self.publish_twist(0.0, 0.0)

    def make_tools(self) -> list:
        """Return Strands @tool functions bound to this controller."""
        ctrl = self

        @tool
        def move_forward(speed_ms: float, duration_s: float) -> str:
            """Move the robot forward at speed_ms (m/s) for duration_s seconds."""
            try:
                speed, duration, warnings = _validate_linear(speed_ms, duration_s)
            except ValueError as e:
                return f"ERROR: {e}"
            ctrl.timed_move(speed, 0.0, duration)
            msg = f"Moving forward at {speed} m/s for {duration} s"
            if warnings:
                msg += " [WARNING: " + "; ".join(warnings) + "]"
            return msg

        @tool
        def move_backward(speed_ms: float, duration_s: float) -> str:
            """Move the robot backward at speed_ms (m/s) for duration_s seconds."""
            try:
                speed, duration, warnings = _validate_linear(speed_ms, duration_s)
            except ValueError as e:
                return f"ERROR: {e}"
            ctrl.timed_move(-speed, 0.0, duration)
            msg = f"Moving backward at {speed} m/s for {duration} s"
            if warnings:
                msg += " [WARNING: " + "; ".join(warnings) + "]"
            return msg

        @tool
        def turn_left(angular_speed_rads: float, duration_s: float) -> str:
            """Turn the robot left (counter-clockwise) at angular_speed_rads (rad/s) for duration_s seconds."""
            try:
                speed, duration, warnings = _validate_angular(angular_speed_rads, duration_s)
            except ValueError as e:
                return f"ERROR: {e}"
            ctrl.timed_move(0.0, speed, duration)
            msg = f"Turning left at {speed} rad/s for {duration} s"
            if warnings:
                msg += " [WARNING: " + "; ".join(warnings) + "]"
            return msg

        @tool
        def turn_right(angular_speed_rads: float, duration_s: float) -> str:
            """Turn the robot right (clockwise) at angular_speed_rads (rad/s) for duration_s seconds."""
            try:
                speed, duration, warnings = _validate_angular(angular_speed_rads, duration_s)
            except ValueError as e:
                return f"ERROR: {e}"
            ctrl.timed_move(0.0, -speed, duration)
            msg = f"Turning right at {speed} rad/s for {duration} s"
            if warnings:
                msg += " [WARNING: " + "; ".join(warnings) + "]"
            return msg

        @tool
        def stop_robot() -> str:
            """Immediately stop all robot motion."""
            ctrl._stop_event.set()
            ctrl.publish_twist(0.0, 0.0)
            return "Robot stopped"

        @tool
        def get_odometry() -> str:
            """Return the robot's current position and heading from odometry."""
            snap = ctrl._odom_cache
            if snap is None:
                return "Odometry not yet available"
            return (
                f"x={snap.x:.3f} m, y={snap.y:.3f} m, theta={snap.theta_deg:.1f} deg, "
                f"linear_vel={snap.linear_vel:.3f} m/s, angular_vel={snap.angular_vel:.3f} rad/s"
            )

        return [move_forward, move_backward, turn_left, turn_right, stop_robot, get_odometry]
