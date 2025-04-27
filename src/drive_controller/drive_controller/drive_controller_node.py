import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import DigitalOutputDevice, PWMOutputDevice

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

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - angular
        right_speed = linear + angular

        self.drive_motor('left', left_speed)
        self.drive_motor('right', right_speed)

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
