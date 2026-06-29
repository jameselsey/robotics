"""Map joystick button presses to voice-agent control events."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoystickVoiceControl(Node):
    def __init__(self):
        super().__init__("joystick_voice_control")
        self.declare_parameter("wake_button", 0)  # Xbox A
        self.declare_parameter("stop_button", 1)  # Xbox B
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("voice_control_topic", "voice_control")

        self.wake_button = int(self.get_parameter("wake_button").value)
        self.stop_button = int(self.get_parameter("stop_button").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        voice_control_topic = str(self.get_parameter("voice_control_topic").value)

        self._previous_buttons: list[int] = []
        self._pub = self.create_publisher(String, voice_control_topic, 10)
        self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        self.get_logger().info(
            f"Joystick voice controls ready: A/button {self.wake_button}=wake, "
            f"B/button {self.stop_button}=stop"
        )

    def _button_pressed(self, buttons: list[int], index: int) -> bool:
        current = index < len(buttons) and buttons[index] == 1
        previous = index < len(self._previous_buttons) and self._previous_buttons[index] == 1
        return current and not previous

    def _publish_control(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._pub.publish(msg)
        self.get_logger().info(f"Published voice control command: {command}")

    def _joy_callback(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        if self._button_pressed(buttons, self.wake_button):
            self._publish_control("wake")
        if self._button_pressed(buttons, self.stop_button):
            self._publish_control("stop")
        self._previous_buttons = buttons


def main(args=None):
    rclpy.init(args=args)
    node = JoystickVoiceControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
