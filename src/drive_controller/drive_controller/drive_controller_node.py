import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf_transformations
import tf2_ros

from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Drive controller with encoders starting...")

        # ---- Parameters ----
        self.declare_parameter('left_in1', 17)
        self.declare_parameter('left_in2', 27)
        self.declare_parameter('right_in3', 26)
        self.declare_parameter('right_in4', 23)
        self.declare_parameter('left_pwm_pin', 22)
        self.declare_parameter('right_pwm_pin', 13)

        # Encoder pins (A/B per wheel). Set to -1 to disable a wheel’s encoder.
        self.declare_parameter('left_enc_a', 24)
        self.declare_parameter('left_enc_b', 25)
        self.declare_parameter('right_enc_a', 5)
        self.declare_parameter('right_enc_b', 6)
        # Debounce for encoder inputs (seconds). Set lower for hall/optical encoders.
        self.declare_parameter('enc_bounce_time', 0.0005)

        # Robot kinematics
        # measure from the axle to the middle of the tread (or use the drive sprocket pitch radius).
        # Example: 52 mm radius → 0.052.
        self.declare_parameter('wheel_radius_m', 0.026)
        # wheel_base_m: measure centerline-to-centerline of the two tracks (not outer edge to outer edge).
        # Example: 280 mm → 0.28
        self.declare_parameter('wheel_base_m', 0.185)
        self.declare_parameter('counts_per_rev', 2048)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_rate_hz', 50.0)

        p = self.get_parameter
        self.left_in1 = DigitalOutputDevice(p('left_in1').value)
        self.left_in2 = DigitalOutputDevice(p('left_in2').value)
        self.right_in3 = DigitalOutputDevice(p('right_in3').value)
        self.right_in4 = DigitalOutputDevice(p('right_in4').value)

        self.left_pwm = PWMOutputDevice(p('left_pwm_pin').value, frequency=100)
        self.right_pwm = PWMOutputDevice(p('right_pwm_pin').value, frequency=100)
        self.left_pwm.value = 0.0
        self.right_pwm.value = 0.0

        # Encoders
        self.wheel_radius = float(p('wheel_radius_m').value)
        self.wheel_base = float(p('wheel_base_m').value)
        self.counts_per_rev = float(p('counts_per_rev').value)
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.counts_per_rev

        la, lb = p('left_enc_a').value, p('left_enc_b').value
        ra, rb = p('right_enc_a').value, p('right_enc_b').value
        bounce = float(p('enc_bounce_time').value)

        self.left_enc = None
        self.right_enc = None
        if la >= 0 and lb >= 0:
            self.left_enc = RotaryEncoder(la, lb, max_steps=0, wrap=False, bounce_time=bounce)
        if ra >= 0 and rb >= 0:
            self.right_enc = RotaryEncoder(ra, rb, max_steps=0, wrap=False, bounce_time=bounce)

        self.last_left_steps = 0
        self.last_right_steps = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value

        self.current_cmd = Twist()
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        rate_hz = float(p('odom_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self.update)

        self.get_logger().info(
            f"Encoders: left={'on' if self.left_enc else 'off'}, right={'on' if self.right_enc else 'off'}, "
            f"m/tick={self.m_per_tick:.6e}, bounce_time={bounce}s"
        )

    def cmd_callback(self, msg: Twist):
        self.current_cmd = msg
        linear = msg.linear.x
        angular = msg.angular.z
        left_speed = linear - (angular * self.wheel_base / 2.0)
        right_speed = linear + (angular * self.wheel_base / 2.0)

        vmax = 0.5
        self.drive_motor('left', left_speed / vmax)
        self.drive_motor('right', right_speed / vmax)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        left_steps = self.left_enc.steps if self.left_enc else None
        right_steps = self.right_enc.steps if self.right_enc else None

        d_left = 0.0
        d_right = 0.0
        if left_steps is not None:
            d_left = (left_steps - self.last_left_steps) * self.m_per_tick
            self.last_left_steps = left_steps
        if right_steps is not None:
            d_right = (right_steps - self.last_right_steps) * self.m_per_tick
            self.last_right_steps = right_steps

        eps = 1e-3
        if self.left_enc and not self.right_enc:
            if abs(self.current_cmd.angular.z) < eps:
                d_right = d_left
        elif self.right_enc and not self.left_enc:
            if abs(self.current_cmd.angular.z) < eps:
                d_left = d_right
        elif (not self.left_enc) and (not self.right_enc):
            v = self.current_cmd.linear.x
            w = self.current_cmd.angular.z
            d_left = (v - w * self.wheel_base / 2.0) * dt
            d_right = (v + w * self.wheel_base / 2.0) * dt

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_base

        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v_meas = d_center / dt
        w_meas = d_theta / dt
        self.publish_odom(now, v_meas, w_meas)
        self.broadcast_tf(now)

    def drive_motor(self, side, pwm_val):
        pwm_val = max(min(pwm_val, 1.0), -1.0)
        direction = pwm_val >= 0
        duty = abs(pwm_val)
        if side == 'left':
            self.left_in1.value = direction
            self.left_in2.value = not direction
            self.left_pwm.value = duty
        elif side == 'right':
            self.right_in3.value = direction
            self.right_in4.value = not direction
            self.right_pwm.value = duty

    def publish_odom(self, stamp, v, w):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    def broadcast_tf(self, stamp):
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.get_logger().info("Stopping motors and cleaning up...")
        self.left_pwm.value = 0.0
        self.right_pwm.value = 0.0
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

if __name__ == '__main__':
    main()
