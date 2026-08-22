#!/usr/bin/env python3
"""Restore and persist AMCL's last known pose for headless localization."""

import json
import math
import os
import tempfile
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node


DEFAULT_COVARIANCE = [0.0] * 36
DEFAULT_COVARIANCE[0] = 0.25
DEFAULT_COVARIANCE[7] = 0.25
DEFAULT_COVARIANCE[35] = 0.0685


class LocalizationPoseManager(Node):
    """Publish a stored seed pose and persist subsequent AMCL estimates."""

    def __init__(self):
        """Create publishers, subscriptions, and the restore timer."""
        super().__init__('localization_pose_manager')
        self.declare_parameter('pose_file', '')
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)

        self.pose_file = os.path.expanduser(
            self.get_parameter('pose_file').get_parameter_value().string_value
        )
        self.initial_pose = self._load_pose()
        self.initial_pose_sent = False
        self.localized = False
        self.started_at = time.monotonic()
        self.last_saved_at = 0.0

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10,
        )
        self.create_timer(2.0, self._publish_initial_pose)

    def _fallback_pose(self):
        yaw = self.get_parameter('initial_pose_yaw').value
        return {
            'x': self.get_parameter('initial_pose_x').value,
            'y': self.get_parameter('initial_pose_y').value,
            'z': 0.0,
            'qx': 0.0,
            'qy': 0.0,
            'qz': math.sin(yaw / 2.0),
            'qw': math.cos(yaw / 2.0),
            'covariance': DEFAULT_COVARIANCE,
        }

    def _load_pose(self):
        try:
            with open(self.pose_file, encoding='utf-8') as pose_stream:
                pose = json.load(pose_stream)
            self.get_logger().info(
                f'Restoring AMCL pose from {self.pose_file}'
            )
            return pose
        except (
            FileNotFoundError,
            json.JSONDecodeError,
            OSError,
            KeyError,
        ) as exc:
            self.get_logger().warning(
                f'No usable saved pose at {self.pose_file} ({exc}); '
                'using configured seed pose'
            )
            return self._fallback_pose()

    def _publish_initial_pose(self):
        if self.localized or time.monotonic() - self.started_at > 60.0:
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = float(self.initial_pose['x'])
        message.pose.pose.position.y = float(self.initial_pose['y'])
        message.pose.pose.position.z = float(self.initial_pose.get('z', 0.0))
        message.pose.pose.orientation.x = float(
            self.initial_pose.get('qx', 0.0)
        )
        message.pose.pose.orientation.y = float(
            self.initial_pose.get('qy', 0.0)
        )
        message.pose.pose.orientation.z = float(self.initial_pose['qz'])
        message.pose.pose.orientation.w = float(self.initial_pose['qw'])
        message.pose.covariance = self.initial_pose.get(
            'covariance', DEFAULT_COVARIANCE
        )
        self.initial_pose_pub.publish(message)
        if not self.initial_pose_sent:
            self.get_logger().info('Published initial pose; waiting for AMCL')
            self.initial_pose_sent = True

    def _amcl_pose_callback(self, message):
        if not self.initial_pose_sent:
            return
        if not self.localized:
            self.localized = True
            self.get_logger().info(
                'AMCL localized; automatic initial-pose publishing stopped'
            )

        now = time.monotonic()
        if now - self.last_saved_at < 2.0:
            return
        self.last_saved_at = now
        pose = message.pose.pose
        state = {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
            'qw': pose.orientation.w,
            'covariance': list(message.pose.covariance),
        }
        self._write_pose(state)

    def _write_pose(self, pose):
        directory = os.path.dirname(self.pose_file)
        try:
            os.makedirs(directory, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=directory, delete=False
            ) as pose_stream:
                json.dump(pose, pose_stream, indent=2)
                pose_stream.write('\n')
                temporary_path = pose_stream.name
            os.replace(temporary_path, self.pose_file)
        except OSError as exc:
            self.get_logger().error(f'Could not save localization pose: {exc}')


def main(args=None):
    """Run the localization pose manager node."""
    rclpy.init(args=args)
    node = LocalizationPoseManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
