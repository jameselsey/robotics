import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math


class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Drive controller starting...")

        # Define motor control pins using gpiozero
        self.left_in1 = DigitalOutputDevice(17)
        self.left_in2 = DigitalOutputDevice(27)
        self.right_in3 = DigitalOutputDevice(22)
        self.right_in4 = DigitalOutputDevice(23)

        self.left_pwm = PWMOutputDevice(18, frequency=100)
        self.right_pwm = PWMOutputDevice(13, frequency=100)

        # Start with motors off
        self.left_pwm.value = 0
        self.right_pwm.value = 0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - angular
        right_speed = linear + angular

        self.drive_motor('left', left_speed)
        self.drive_motor('right', right_speed)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # seconds
        self.last_time = current_time

        # Basic dead-reckoning based on velocities
        delta_x = linear * math.cos(self.theta) * dt
        delta_y = linear * math.sin(self.theta) * dt
        delta_theta = angular * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = linear
        odom.twist.twist.angular.z = angular

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def drive_motor(self, side, speed):
        direction = speed >= 0
        duty = min(abs(speed), 1.0)  # gpiozero PWM value is 0.0–1.0

        if side == 'left':
            self.left_in1.value = direction
            self.left_in2.value = not direction
            self.left_pwm.value = duty
        elif side == 'right':
            self.right_in3.value = direction
            self.right_in4.value = not direction
            self.right_pwm.value = duty

    def destroy_node(self):
        self.get_logger().info("Stopping motors and cleaning up...")
        self.left_pwm.value = 0
        self.right_pwm.value = 0
        # Devices will auto-cleanup on exit
        super().destroy_node()

def main():
    rclpy.init()
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
