"""Semantic map tools for room labels and Nav2 goals."""

import math
from pathlib import Path
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from strands import tool
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover - nav2_msgs may be absent on development machines
    NavigateToPose = None


def _yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


def _point_in_polygon(x: float, y: float, polygon: list[list[float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, point in enumerate(polygon):
        xi, yi = float(point[0]), float(point[1])
        xj, yj = float(polygon[j][0]), float(polygon[j][1])
        crosses = (yi > y) != (yj > y)
        if crosses:
            denom = yj - yi
            if abs(denom) < 1e-12:
                j = i
                continue
            x_intersection = (xj - xi) * (y - yi) / denom + xi
            if x < x_intersection:
                inside = not inside
        j = i
    return inside


def _polygon_centroid(polygon: list[list[float]]) -> tuple[float, float]:
    """Return a useful centre point for a room polygon in map-frame metres."""
    if not polygon:
        raise ValueError("Polygon is empty")

    # Use the area-weighted centroid when the polygon is well-formed. Fall back to
    # the average of vertices for tiny or degenerate polygons.
    area_twice = 0.0
    centroid_x = 0.0
    centroid_y = 0.0
    points = [(float(point[0]), float(point[1])) for point in polygon]
    for index, (x0, y0) in enumerate(points):
        x1, y1 = points[(index + 1) % len(points)]
        cross = x0 * y1 - x1 * y0
        area_twice += cross
        centroid_x += (x0 + x1) * cross
        centroid_y += (y0 + y1) * cross

    if abs(area_twice) < 1e-9:
        return (
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
        )

    return centroid_x / (3.0 * area_twice), centroid_y / (3.0 * area_twice)


class SemanticMapController:
    """Connects SLAM's map frame to human room labels and Nav2 goals."""

    def __init__(self, ros_node, config_path: str):
        self._node = ros_node
        self._config_path = Path(config_path)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, ros_node)
        self._nav_client = None
        if NavigateToPose is not None:
            self._nav_client = ActionClient(ros_node, NavigateToPose, "navigate_to_pose")
        self._load_config()

    def _load_config(self) -> None:
        if not self._config_path.exists():
            self._node.get_logger().warn(f"Room config not found: {self._config_path}")
            self._config = {"frame_id": "map", "base_frame": "base_link", "rooms": {}}
            return
        with self._config_path.open() as f:
            self._config = yaml.safe_load(f) or {}
        self._config.setdefault("frame_id", "map")
        self._config.setdefault("base_frame", "base_link")
        self._config.setdefault("rooms", {})

    @property
    def map_frame(self) -> str:
        return str(self._config.get("frame_id") or "map")

    @property
    def base_frame(self) -> str:
        return str(self._config.get("base_frame") or "base_link")

    def _current_pose_xy(self) -> tuple[float, float]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            raise RuntimeError(f"No {self.map_frame}->{self.base_frame} transform yet: {exc}") from exc
        t = transform.transform.translation
        return float(t.x), float(t.y)

    def _find_room(self, x: float, y: float) -> str | None:
        for name, room in (self._config.get("rooms") or {}).items():
            polygon = room.get("polygon") or []
            if len(polygon) >= 3 and _point_in_polygon(x, y, polygon):
                return str(name)
        return None

    def _room_key(self, room_name: str) -> str | None:
        requested = room_name.strip().lower()
        rooms = self._config.get("rooms") or {}
        for name in rooms:
            if str(name).lower() == requested:
                return str(name)
        for name in rooms:
            if requested and requested in str(name).lower():
                return str(name)
        return None

    def _room_goal(self, room_name: str, room: dict) -> tuple[float, float, float, str]:
        pose = room.get("navigate_pose") or {}
        if {"x", "y"}.issubset(pose):
            return (
                float(pose["x"]),
                float(pose["y"]),
                float(pose.get("yaw", 0.0)),
                "navigate_pose",
            )

        polygon = room.get("polygon") or []
        if len(polygon) < 3:
            raise ValueError(
                f"Room '{room_name}' does not have a navigate_pose or a valid polygon."
            )
        x, y = _polygon_centroid(polygon)
        return x, y, 0.0, "polygon centroid"

    def _room_summary(self, room_name: str, room: dict) -> str:
        polygon = room.get("polygon") or []
        parts = [f"{room_name}: {len(polygon)} polygon points"]
        try:
            x, y, yaw, source = self._room_goal(room_name, room)
            parts.append(f"goal from {source} at x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        except ValueError:
            parts.append("no navigation goal")
        return "; ".join(parts)

    def make_tools(self) -> list:
        ctrl = self

        @tool
        def reload_room_labels() -> str:
            """Reload room label polygons and navigation poses from the room YAML file."""
            ctrl._load_config()
            return f"Loaded {len(ctrl._config.get('rooms') or {})} room labels from {ctrl._config_path}"

        @tool
        def what_room_am_i_in() -> str:
            """Return the robot's current labelled room using SLAM's map frame and room polygons."""
            try:
                x, y = ctrl._current_pose_xy()
            except RuntimeError as exc:
                return f"ERROR: {exc}"
            room = ctrl._find_room(x, y)
            if room is None:
                return f"Current map pose is x={x:.2f}, y={y:.2f}, but that point is not inside any labelled room."
            return f"Current map pose is x={x:.2f}, y={y:.2f}; labelled room: {room}."

        @tool
        def list_known_rooms() -> str:
            """List the annotated rooms currently available to the robot."""
            rooms = sorted((ctrl._config.get("rooms") or {}).keys())
            if not rooms:
                return "No rooms are labelled yet. Add polygons and optional navigate_pose entries to rooms.yaml."
            return "Known rooms: " + ", ".join(str(room) for room in rooms)

        @tool
        def describe_room_annotations() -> str:
            """Describe all annotated rooms, polygon sizes, and available navigation targets."""
            rooms = ctrl._config.get("rooms") or {}
            if not rooms:
                return "No rooms are labelled yet. Add polygons and optional navigate_pose entries to rooms.yaml."
            summaries = [
                ctrl._room_summary(str(name), room or {})
                for name, room in sorted(rooms.items())
            ]
            return "Room annotations:\n" + "\n".join(f"- {summary}" for summary in summaries)

        @tool
        def get_room_details(room_name: str) -> str:
            """Return details about one annotated room by name."""
            rooms = ctrl._config.get("rooms") or {}
            key = ctrl._room_key(room_name)
            if key is None:
                return f"ERROR: Unknown room '{room_name}'. Try list_known_rooms first."
            return ctrl._room_summary(key, rooms[key] or {})

        @tool
        def where_am_i_on_the_map() -> str:
            """Return the robot's current map-frame pose coordinates and labelled room if available."""
            try:
                x, y = ctrl._current_pose_xy()
            except RuntimeError as exc:
                return f"ERROR: {exc}"
            room = ctrl._find_room(x, y)
            if room is None:
                return f"Current map pose: x={x:.2f}, y={y:.2f}. This is not inside a labelled room."
            return f"Current map pose: x={x:.2f}, y={y:.2f}. Labelled room: {room}."

        @tool
        def navigate_to_room(room_name: str) -> str:
            """Send a Nav2 NavigateToPose goal for a labelled room. Uses navigate_pose, or the room polygon centroid if no pose is set."""
            rooms = ctrl._config.get("rooms") or {}
            key = ctrl._room_key(room_name)
            if key is None:
                return f"ERROR: Unknown room '{room_name}'. Try list_known_rooms first."
            room = rooms[key] or {}
            try:
                x, y, yaw, source = ctrl._room_goal(key, room)
            except ValueError as exc:
                return f"ERROR: {exc}"
            if ctrl._nav_client is None:
                return "ERROR: nav2_msgs is not installed, so I cannot send Nav2 goals."
            if not ctrl._nav_client.wait_for_server(timeout_sec=2.0):
                return "ERROR: Nav2 navigate_to_pose action server is not running yet."

            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = ctrl.map_frame
            goal.pose.header.stamp = ctrl._node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            qz, qw = _yaw_to_quaternion(yaw)
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw

            future = ctrl._nav_client.send_goal_async(goal)

            def log_goal_response(done_future):
                try:
                    handle = done_future.result()
                    if handle.accepted:
                        ctrl._node.get_logger().info(f"Nav2 accepted room goal for {key}")
                    else:
                        ctrl._node.get_logger().warn(f"Nav2 rejected room goal for {key}")
                except Exception as exc:
                    ctrl._node.get_logger().warn(f"Nav2 goal response failed for {key}: {exc}")

            future.add_done_callback(log_goal_response)
            return (
                f"Sent Nav2 goal for {key} using {source}: "
                f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}."
            )

        return [
            reload_room_labels,
            what_room_am_i_in,
            where_am_i_on_the_map,
            list_known_rooms,
            describe_room_annotations,
            get_room_details,
            navigate_to_room,
        ]
