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
MAX_DURATION_S = 15.0
MAX_DISTANCE_M = 3.0
DISTANCE_MONITOR_HZ = 20.0
DISTANCE_TOLERANCE_M = 0.03
MAX_SPIN_DURATION_S = 30.0
SPIN_MONITOR_HZ = 20.0
SPIN_TOLERANCE_DEG = 8.0
DEFAULT_LINEAR_SPEED_MS = 0.25
DEFAULT_DISTANCE_SPEED_MS = 0.18
DEFAULT_TURN_ANGULAR_SPEED_RADS = 0.5
DEFAULT_SPIN_ANGULAR_SPEED_RADS = 0.35
DEFAULT_TURN_DURATION_S = 1.5
DEFAULT_SPIN_DEGREES = 360.0


def _angle_delta_degrees(current: float, previous: float) -> float:
    return (current - previous + 180.0) % 360.0 - 180.0


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


def _validate_distance(distance_m: float, speed_ms: float):
    if distance_m <= 0.0:
        raise ValueError("distance_m must be > 0.0")
    if speed_ms <= 0.0:
        raise ValueError("speed_ms must be > 0.0")
    warnings = []
    if distance_m > MAX_DISTANCE_M:
        warnings.append(f"distance clamped from {distance_m:.2f} to {MAX_DISTANCE_M} m")
        distance_m = MAX_DISTANCE_M
    if speed_ms > MAX_LINEAR_SPEED_MS:
        warnings.append(f"speed clamped from {speed_ms:.3f} to {MAX_LINEAR_SPEED_MS} m/s")
        speed_ms = MAX_LINEAR_SPEED_MS
    return distance_m, speed_ms, warnings

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

    def drive_distance(self, linear_x: float, distance_m: float, timeout_s: float) -> tuple[bool, float, float]:
        self._stop_event.clear()
        snap = self._odom_cache
        if snap is None:
            self.timed_move(linear_x, 0.0, timeout_s)
            return False, 0.0, timeout_s

        start_x = snap.x
        start_y = snap.y
        started_at = time.monotonic()
        deadline = started_at + timeout_s
        period_s = 1.0 / DISTANCE_MONITOR_HZ

        self.publish_twist(linear_x, 0.0)
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            self._stop_event.wait(timeout=period_s)
            latest = self._odom_cache
            if latest is None:
                continue
            travelled = math.hypot(latest.x - start_x, latest.y - start_y)
            if travelled >= max(0.0, distance_m - DISTANCE_TOLERANCE_M):
                self.publish_twist(0.0, 0.0)
                return True, travelled, time.monotonic() - started_at

        self.publish_twist(0.0, 0.0)
        latest = self._odom_cache
        travelled = 0.0 if latest is None else math.hypot(latest.x - start_x, latest.y - start_y)
        return False, travelled, time.monotonic() - started_at

    def spin_by_degrees(self, angular_z: float, degrees: float, timeout_s: float) -> tuple[bool, float, float]:
        self._stop_event.clear()
        snap = self._odom_cache
        if snap is None:
            self.timed_move(0.0, angular_z, timeout_s)
            return False, 0.0, timeout_s

        target = abs(degrees)
        accumulated = 0.0
        previous_theta = snap.theta_deg
        started_at = time.monotonic()
        deadline = started_at + timeout_s
        period_s = 1.0 / SPIN_MONITOR_HZ

        self.publish_twist(0.0, angular_z)
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            self._stop_event.wait(timeout=period_s)
            latest = self._odom_cache
            if latest is None:
                continue
            delta = _angle_delta_degrees(latest.theta_deg, previous_theta)
            previous_theta = latest.theta_deg
            accumulated += abs(delta)
            if accumulated >= target - SPIN_TOLERANCE_DEG:
                self.publish_twist(0.0, 0.0)
                return True, accumulated, time.monotonic() - started_at

        self.publish_twist(0.0, 0.0)
        return False, accumulated, time.monotonic() - started_at

    def make_tools(self) -> list:
        """Return Strands @tool functions bound to this controller."""
        ctrl = self

        @tool
        def move_forward(speed_ms: float = DEFAULT_LINEAR_SPEED_MS, duration_s: float = 1.0) -> str:
            """Move the robot forward. Use defaults for casual requests unless the user gives a speed or duration."""
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
        def move_backward(speed_ms: float = DEFAULT_LINEAR_SPEED_MS, duration_s: float = 1.0) -> str:
            """Move the robot backward. Use defaults for casual requests unless the user gives a speed or duration."""
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
        def drive_forward_distance(distance_m: float = 1.0, speed_ms: float = DEFAULT_DISTANCE_SPEED_MS) -> str:
            """Drive forward by a measured distance in metres using odometry. Use this when the user asks to drive forward a distance such as 1 metre."""
            try:
                distance, speed, warnings = _validate_distance(distance_m, speed_ms)
            except ValueError as e:
                return f"ERROR: {e}"
            timeout_s = min(MAX_DURATION_S, max(distance / speed * 2.0, distance / speed + 3.0))
            completed, actual_distance, elapsed_s = ctrl.drive_distance(speed, distance, timeout_s)
            if completed:
                msg = f"Drove forward {actual_distance:.2f} m in {elapsed_s:.2f} s"
            elif ctrl._odom_cache is None:
                msg = f"Drove forward using timed fallback for {elapsed_s:.2f} s; odometry was not available"
            else:
                msg = f"Forward drive timed out after {elapsed_s:.2f} s at about {actual_distance:.2f} m; distance calibration needs attention"
            if warnings:
                msg += " [WARNING: " + "; ".join(warnings) + "]"
            return msg

        @tool
        def drive_backward_distance(distance_m: float = 1.0, speed_ms: float = DEFAULT_DISTANCE_SPEED_MS) -> str:
            """Drive backward by a measured distance in metres using odometry. Use this when the user asks to reverse a distance such as 1 metre."""
            try:
                distance, speed, warnings = _validate_distance(distance_m, speed_ms)
            except ValueError as e:
                return f"ERROR: {e}"
            timeout_s = min(MAX_DURATION_S, max(distance / speed * 2.0, distance / speed + 3.0))
            completed, actual_distance, elapsed_s = ctrl.drive_distance(-speed, distance, timeout_s)
            if completed:
                msg = f"Drove backward {actual_distance:.2f} m in {elapsed_s:.2f} s"
            elif ctrl._odom_cache is None:
                msg = f"Drove backward using timed fallback for {elapsed_s:.2f} s; odometry was not available"
            else:
                msg = f"Backward drive timed out after {elapsed_s:.2f} s at about {actual_distance:.2f} m; distance calibration needs attention"
            if warnings:
                msg += " [WARNING: " + "; ".join(warnings) + "]"
            return msg

        @tool
        def turn_left(angular_speed_rads: float = DEFAULT_TURN_ANGULAR_SPEED_RADS, duration_s: float = DEFAULT_TURN_DURATION_S) -> str:
            """Turn the robot left briefly. For a full in-place rotation, use spin_on_the_spot instead."""
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
        def turn_right(angular_speed_rads: float = DEFAULT_TURN_ANGULAR_SPEED_RADS, duration_s: float = DEFAULT_TURN_DURATION_S) -> str:
            """Turn the robot right briefly. For a full in-place rotation, use spin_on_the_spot instead."""
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
        def spin_on_the_spot(direction: str = "left", degrees: float = DEFAULT_SPIN_DEGREES, angular_speed_rads: float = DEFAULT_SPIN_ANGULAR_SPEED_RADS) -> str:
            """Spin on the spot/in place by rotating the wheels in opposite directions. Use for 'spin on a spot', 'spin on the spot', 'do a 360', or 'turn around fully'."""
            direction_normalized = direction.strip().lower() if isinstance(direction, str) else "left"
            if direction_normalized in ("clockwise", "right", "cw"):
                sign = -1.0
                direction_label = "right"
            elif direction_normalized in ("counter-clockwise", "counterclockwise", "left", "ccw"):
                sign = 1.0
                direction_label = "left"
            else:
                return "ERROR: direction must be left/right, clockwise/counter-clockwise, cw, or ccw"
            if degrees <= 0.0:
                return "ERROR: degrees must be > 0.0"
            try:
                speed, _, warnings = _validate_angular(angular_speed_rads, 1.0)
            except ValueError as e:
                return f"ERROR: {e}"
            estimated_duration_s = math.radians(degrees) / speed
            timeout_s = min(MAX_SPIN_DURATION_S, max(estimated_duration_s * 2.0, estimated_duration_s + 3.0))
            completed, actual_degrees, elapsed_s = ctrl.spin_by_degrees(sign * speed, degrees, timeout_s)
            if completed:
                msg = f"Spun on the spot {direction_label} by {actual_degrees:.0f} degrees in {elapsed_s:.2f} s"
            elif ctrl._odom_cache is None:
                msg = f"Spun on the spot {direction_label} using timed fallback for {elapsed_s:.2f} s; odometry was not available"
            else:
                msg = f"Spin timed out after {elapsed_s:.2f} s at about {actual_degrees:.0f} degrees; movement calibration needs attention"
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

        return [move_forward, move_backward, drive_forward_distance, drive_backward_distance, spin_on_the_spot, turn_left, turn_right, stop_robot, get_odometry]
