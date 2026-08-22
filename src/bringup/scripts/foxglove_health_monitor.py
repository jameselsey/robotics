#!/usr/bin/env python3
"""Publish health for the ROS inputs required by Foxglove's robot panels."""

import json
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


STARTUP_GRACE_SECONDS = 12.0
TOPIC_STALE_SECONDS = 2.0


class FoxgloveHealthMonitor(Node):
    """Check topic freshness and both transform chains used by Foxglove."""

    def __init__(self):
        super().__init__('foxglove_health_monitor')
        self.started_at = time.monotonic()
        self.last_seen = {}
        self.last_summary = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            LaserScan, '/scan', self._mark_seen('/scan'), qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom', self._mark_seen('/odom'), 10)
        self.create_subscription(
            String,
            '/robot_description',
            self._mark_seen('/robot_description'),
            transient_qos,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._mark_seen('/map'), transient_qos
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.health_pub = self.create_publisher(
            String, '/foxglove_health', transient_qos
        )
        self.create_timer(2.0, self._publish_health)

    def _mark_seen(self, topic):
        def callback(_message):
            self.last_seen[topic] = time.monotonic()

        return callback

    def _topic_check(self, topic, now, *, latched=False):
        seen_at = self.last_seen.get(topic)
        if seen_at is None:
            return False, 'no message received'
        age = now - seen_at
        if latched:
            return True, f'received ({age:.1f}s ago; latched topic)'
        if age > TOPIC_STALE_SECONDS:
            return False, f'last message was {age:.1f}s ago'
        return True, f'fresh ({age:.1f}s old)'

    def _tf_check(self, target, source):
        available = self.tf_buffer.can_transform(
            target, source, rclpy.time.Time(), timeout=Duration(seconds=0.0)
        )
        if available:
            return True, 'available'
        return False, f'missing {target} <- {source}'

    @staticmethod
    def _diagnostic(name, ok, detail, starting):
        status = DiagnosticStatus()
        status.name = f'robopi/foxglove/{name}'
        status.hardware_id = 'robopi'
        if ok:
            status.level = DiagnosticStatus.OK
            status.message = detail
        elif starting:
            status.level = DiagnosticStatus.WARN
            status.message = f'starting: {detail}'
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = detail
        status.values = [KeyValue(key='detail', value=detail)]
        return status

    def _publish_health(self):
        now = time.monotonic()
        starting = now - self.started_at < STARTUP_GRACE_SECONDS
        checks = {
            '/scan': self._topic_check('/scan', now),
            '/odom': self._topic_check('/odom', now),
            '/robot_description': self._topic_check(
                '/robot_description', now, latched=True
            ),
            '/map': self._topic_check('/map', now, latched=True),
        }
        checks['tf:base_link<-laser'] = self._tf_check('base_link', 'laser')
        checks['tf:map<-base_link'] = self._tf_check('map', 'base_link')

        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.status = [
            self._diagnostic(name, ok, detail, starting)
            for name, (ok, detail) in checks.items()
        ]
        self.diagnostics_pub.publish(diagnostic_array)

        failures = [name for name, (ok, _detail) in checks.items() if not ok]
        overall = 'STARTING' if starting and failures else ('ERROR' if failures else 'OK')
        summary = {
            'status': overall,
            'failures': failures,
            'checks': {
                name: {'ok': ok, 'detail': detail}
                for name, (ok, detail) in checks.items()
            },
        }
        message = String()
        message.data = json.dumps(summary, sort_keys=True)
        self.health_pub.publish(message)

        summary_key = (overall, tuple(failures))
        if summary_key != self.last_summary:
            message_text = (
                f'Foxglove inputs: {overall}; failures={failures or "none"}'
            )
            if overall == 'OK':
                self.get_logger().info(message_text)
            else:
                self.get_logger().warning(message_text)
            self.last_summary = summary_key


def main(args=None):
    rclpy.init(args=args)
    node = FoxgloveHealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
