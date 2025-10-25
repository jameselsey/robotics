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
        self.get_logger().info("Drive controller with encoders starting (BTS7960 mode)...")

        # ---- Parameters ----
        # Reusing your existing pins
        # LEFT: RPWM=22, LPWM=17, EN=27
        # RIGHT: RPWM=13, LPWM=26, EN=23
        self.declare_parameter('left_rpwm_pin', 22)
        self.declare_parameter('left_lpwm_pin', 17)
        self.declare_parameter('left_r_en_pin', -1)   # set -1 if hard-wired to 5V
        self.declare_parameter('left_l_en_pin', -1)   # OK to tie both ENs together

        self.declare_parameter('right_rpwm_pin', 13)
        self.declare_parameter('right_lpwm_pin', 26)
        self.declare_parameter('right_r_en_pin', -1)  # set -1 if hard-wired to 5V
        self.declare_parameter('right_l_en_pin', -1)

        # Encoders
        self.declare_parameter('left_enc_a', 24)
        self.declare_parameter('left_enc_b', 25)
        self.declare_parameter('right_enc_a', 5)
        self.declare_parameter('right_enc_b', 6)
        self.declare_parameter('enc_bounce_time', 0.0005)

        # Kinematics
        self.declare_parameter('wheel_radius_m', 0.026)
        self.declare_parameter('wheel_base_m', 0.185)
        self.declare_parameter('counts_per_rev', 2048)

        # Odom / frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_rate_hz', 50.0)

        # Motion mapping & tuning
        self.declare_parameter('vmax', 0.5)            # m/s -> 100% PWM at this speed
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('left_enc_invert', False)
        self.declare_parameter('right_enc_invert', False)
        self.declare_parameter('log_steps', False)

        p = self.get_parameter

        # ---- BTS7960 IO objects ----
        freq = 500  # Hz (bump later if you switch to pigpio for 15–20 kHz)
        self.left_rpwm = PWMOutputDevice(p('left_rpwm_pin').value, frequency=freq)
        self.left_lpwm = PWMOutputDevice(p('left_lpwm_pin').value, frequency=freq)
        self.right_rpwm = PWMOutputDevice(p('right_rpwm_pin').value, frequency=freq)
        self.right_lpwm = PWMOutputDevice(p('right_lpwm_pin').value, frequency=freq)

        # Optional EN pins (drive HIGH if provided)
        self.left_r_en = DigitalOutputDevice(p('left_r_en_pin').value) if p('left_r_en_pin').value >= 0 else None
        self.left_l_en = DigitalOutputDevice(p('left_l_en_pin').value) if p('left_l_en_pin').value >= 0 else None
        self.right_r_en = DigitalOutputDevice(p('right_r_en_pin').value) if p('right_r_en_pin').value >= 0 else None
        self.right_l_en = DigitalOutputDevice(p('right_l_en_pin').value) if p('right_l_en_pin').value >= 0 else None

        for en in (self.left_r_en, self.left_l_en, self.right_r_en, self.right_l_en):
            if en is not None:
                en.on()  # enable bridge

        # Zero PWMs
        self.left_rpwm.value = 0.0
        self.left_lpwm.value = 0.0
        self.right_rpwm.value = 0.0
        self.right_lpwm.value = 0.0

        # Encoders & kinematics
        self.wheel_radius = float(p('wheel_radius_m').value)
        self.wheel_base = float(p('wheel_base_m').value)
        self.counts_per_rev = float(p('counts_per_rev').value)
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.counts_per_rev

        la, lb = p('left_enc_a').value, p('left_enc_b').value
        ra, rb = p('right_enc_a').value, p('right_enc_b').value
        bounce = float(p('enc_bounce_time').value)

        self.left_invert = bool(p('left_enc_invert').value)
        self.right_invert = bool(p('right_enc_invert').value)
        self.log_steps = bool(p('log_steps').value)

        self.left_enc = RotaryEncoder(la, lb, max_steps=0, wrap=False, bounce_time=bounce) if la >= 0 and lb >= 0 else None
        self.right_enc = RotaryEncoder(ra, rb, max_steps=0, wrap=False, bounce_time=bounce) if ra >= 0 and rb >= 0 else None

        # State
        self.last_left_steps = 0
        self.last_right_steps = 0
        self.x = 0.0; self.y = 0.0; self.theta = 0.0

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
            f"BTS7960 pins L(RPWM,LPWM,EN)={(p('left_rpwm_pin').value, p('left_lpwm_pin').value, p('left_r_en_pin').value)} "
            f"R(RPWM,LPWM,EN)={(p('right_rpwm_pin').value, p('right_lpwm_pin').value, p('right_r_en_pin').value)}; "
            f"m_per_tick={self.m_per_tick:.6e}, bounce_time={bounce}s"
        )

    # ----------------- Control -----------------
    def cmd_callback(self, msg: Twist):
        self.current_cmd = msg

        vmax = float(self.get_parameter('vmax').value)
        gL   = float(self.get_parameter('linear_gain').value)
        gA   = float(self.get_parameter('angular_gain').value)

        linear  = gL * msg.linear.x      # m/s
        angular = gA * msg.angular.z     # rad/s

        half_track = self.wheel_base / 2.0
        left_speed  = linear - (angular * half_track)
        right_speed = linear + (angular * half_track)

        # Clamp to [-vmax, +vmax]
        left_speed  = max(min(left_speed,  vmax), -vmax)
        right_speed = max(min(right_speed, vmax), -vmax)

        # To PWM [-1..1]
        pwm_left  = 0.0 if vmax <= 0 else left_speed  / vmax
        pwm_right = 0.0 if vmax <= 0 else right_speed / vmax

        self.drive_motor('left',  pwm_left)
        self.drive_motor('right', pwm_right)

    # ----------------- Odometry -----------------
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now

        left_steps = self.left_enc.steps if self.left_enc else None
        right_steps = self.right_enc.steps if self.right_enc else None

        d_left = 0.0; d_right = 0.0
        if left_steps is not None:
            dl = left_steps - self.last_left_steps
            if self.left_invert: dl = -dl
            d_left = dl * self.m_per_tick
            self.last_left_steps = left_steps
        if right_steps is not None:
            dr = right_steps - self.last_right_steps
            if self.right_invert: dr = -dr
            d_right = dr * self.m_per_tick
            self.last_right_steps = right_steps

        if (self.left_enc is None) and (self.right_enc is None):
            v = self.current_cmd.linear.x; w = self.current_cmd.angular.z
            half_track = self.wheel_base / 2.0
            d_left  = (v - w * half_track) * dt
            d_right = (v + w * half_track) * dt
        elif (self.left_enc is None) and (self.right_enc is not None):
            d_left = d_right
        elif (self.right_enc is None) and (self.left_enc is not None):
            d_right = d_left

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_base if self.wheel_base != 0 else 0.0

        self.theta = math.atan2(math.sin(self.theta + d_theta), math.cos(self.theta + d_theta))
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        v_meas = d_center / dt
        w_meas = d_theta / dt
        self.publish_odom(now, v_meas, w_meas)
        self.broadcast_tf(now)

        if self.log_steps:
            self.get_logger().info(f"steps L={left_steps} R={right_steps}  dL={d_left:.4f} dR={d_right:.4f}  v={v_meas:.3f} w={w_meas:.3f}")

    # ----------------- Hardware helpers (BTS7960) -----------------
    def drive_motor(self, side, pwm_val):
        """pwm_val in [-1, 1]; uses RPWM for +, LPWM for -; EN kept high."""
        pwm_val = max(min(pwm_val, 1.0), -1.0)
        duty = abs(pwm_val)

        if side == 'left':
            if pwm_val >= 0:
                self.left_rpwm.value = duty
                self.left_lpwm.value = 0.0
            else:
                self.left_rpwm.value = 0.0
                self.left_lpwm.value = duty
        elif side == 'right':
            if pwm_val >= 0:
                self.right_rpwm.value = duty
                self.right_lpwm.value = 0.0
            else:
                self.right_rpwm.value = 0.0
                self.right_lpwm.value = duty

    # ----------------- Publishing -----------------
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
        # Coast both sides
        self.left_rpwm.value = 0.0; self.left_lpwm.value = 0.0
        self.right_rpwm.value = 0.0; self.right_lpwm.value = 0.0
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
