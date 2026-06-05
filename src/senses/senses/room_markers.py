"""Publish annotated room polygons as Foxglove/RViz marker arrays."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


ROOM_COLORS = [
    (0.18, 0.63, 0.95),
    (0.95, 0.42, 0.38),
    (0.43, 0.78, 0.39),
    (0.96, 0.74, 0.30),
    (0.68, 0.48, 0.90),
    (0.33, 0.80, 0.74),
]


class RoomMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("room_marker_publisher")

        default_config = (
            Path(get_package_share_directory("senses")) / "config" / "rooms.yaml"
        )
        self.declare_parameter("rooms_config_path", str(default_config))
        self.declare_parameter("marker_topic", "/visualization_marker_array")
        self.declare_parameter("publish_period_sec", 2.0)

        self._rooms_config_path = Path(
            str(self.get_parameter("rooms_config_path").value)
        ).expanduser()
        self._marker_topic = str(self.get_parameter("marker_topic").value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            MarkerArray,
            self._marker_topic,
            qos,
        )

        period = float(self.get_parameter("publish_period_sec").value)
        self._timer = self.create_timer(period, self._publish_markers)
        self.get_logger().info(
            f"Publishing room markers from {self._rooms_config_path} "
            f"to {self._marker_topic}"
        )
        self._publish_markers()

    def _load_config(self) -> dict:
        if not self._rooms_config_path.exists():
            self.get_logger().warn(
                f"Room config does not exist: {self._rooms_config_path}"
            )
            return {"frame_id": "map", "rooms": {}}

        with self._rooms_config_path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream) or {}

    def _publish_markers(self) -> None:
        config = self._load_config()
        frame_id = str(config.get("frame_id") or "map")
        rooms = config.get("rooms") or {}

        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker(frame_id))

        marker_id = 1
        for color_index, (room_name, room) in enumerate(sorted(rooms.items())):
            polygon = room.get("polygon") or []
            if len(polygon) < 3:
                continue

            color = ROOM_COLORS[color_index % len(ROOM_COLORS)]
            marker_array.markers.append(
                self._polygon_marker(marker_id, frame_id, room_name, polygon, color)
            )
            marker_id += 1

            label_point = self._centroid(polygon)
            marker_array.markers.append(
                self._label_marker(marker_id, frame_id, room_name, label_point, color)
            )
            marker_id += 1

            navigate_pose = room.get("navigate_pose") or {}
            if "x" in navigate_pose and "y" in navigate_pose:
                marker_array.markers.append(
                    self._goal_marker(marker_id, frame_id, room_name, navigate_pose, color)
                )
                marker_id += 1

        self._publisher.publish(marker_array)

    def _delete_all_marker(self, frame_id: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def _base_marker(
        self,
        marker_id: int,
        frame_id: str,
        room_name: str,
        marker_type: int,
        color: tuple[float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"rooms/{room_name}"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.9
        return marker

    def _polygon_marker(
        self,
        marker_id: int,
        frame_id: str,
        room_name: str,
        polygon: list,
        color: tuple[float, float, float],
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, room_name, Marker.LINE_STRIP, color)
        marker.scale.x = 0.06
        marker.points = [self._point(point) for point in polygon]
        marker.points.append(self._point(polygon[0]))
        return marker

    def _label_marker(
        self,
        marker_id: int,
        frame_id: str,
        room_name: str,
        label_point: Point,
        color: tuple[float, float, float],
    ) -> Marker:
        marker = self._base_marker(
            marker_id, frame_id, room_name, Marker.TEXT_VIEW_FACING, color
        )
        marker.pose.position = label_point
        marker.pose.position.z = 0.35
        marker.scale.z = 0.35
        marker.text = room_name
        marker.color.a = 1.0
        return marker

    def _goal_marker(
        self,
        marker_id: int,
        frame_id: str,
        room_name: str,
        navigate_pose: dict,
        color: tuple[float, float, float],
    ) -> Marker:
        marker = self._base_marker(marker_id, frame_id, room_name, Marker.ARROW, color)
        marker.pose.position.x = float(navigate_pose.get("x", 0.0))
        marker.pose.position.y = float(navigate_pose.get("y", 0.0))
        marker.pose.position.z = 0.05
        yaw = float(navigate_pose.get("yaw", 0.0))
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.45
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.a = 0.8
        return marker

    def _point(self, value: list | tuple) -> Point:
        point = Point()
        point.x = float(value[0])
        point.y = float(value[1])
        point.z = float(value[2]) if len(value) > 2 else 0.03
        return point

    def _centroid(self, polygon: list) -> Point:
        point = Point()
        point.x = sum(float(vertex[0]) for vertex in polygon) / len(polygon)
        point.y = sum(float(vertex[1]) for vertex in polygon) / len(polygon)
        return point


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RoomMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
