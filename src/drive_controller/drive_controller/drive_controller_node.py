import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Drive controller starting...")

        # Define motor control pins
        self.LEFT_IN1 = 17
        self.LEFT_IN2 = 27
        self.RIGHT_IN3 = 22
        self.RIGHT_IN4 = 23
        self.LEFT_PWM = 18
        self.RIGHT_PWM = 13

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.LEFT_IN1, self.LEFT_IN2, self.RIGHT_IN3, self.RIGHT_IN4], GPIO.OUT)
        GPIO.setup([self.LEFT_PWM, self.RIGHT_PWM], GPIO.OUT)

        self.pwm_left = GPIO.PWM(self.LEFT_PWM, 100)
        self.pwm_right = GPIO.PWM(self.RIGHT_PWM, 100)

        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left_speed = linear - angular
        right_speed = linear + angular

        self.drive_motor('left', left_speed)
        self.drive_motor('right', right_speed)

    def drive_motor(self, side, speed):
        direction = GPIO.HIGH if speed >= 0 else GPIO.LOW
        duty = min(abs(speed) * 100, 100)

        if side == 'left':
            GPIO.output(self.LEFT_IN1, direction)
            GPIO.output(self.LEFT_IN2, not direction)
            self.pwm_left.ChangeDutyCycle(duty)
        elif side == 'right':
            GPIO.output(self.RIGHT_IN3, direction)
            GPIO.output(self.RIGHT_IN4, not direction)
            self.pwm_right.ChangeDutyCycle(duty)

    def destroy_node(self):
        self.get_logger().info("Stopping motors and cleaning up...")
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
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
