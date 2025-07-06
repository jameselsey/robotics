import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class Mouth(Node):
    def __init__(self):
        super().__init__('mouth')
        self.get_logger().info('👄 Mouth node has been started.')

        # Subscribe to brain's output
        self.subscription = self.create_subscription(
            String,
            'speech_output',
            self.say_text,
            10
        )

        # Initialize TTS engine
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 180)  # adjust speech speed if needed

    def say_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().info("🤐 Received empty message. Skipping TTS.")
            return

        self.get_logger().info(f"🗣️ Speaking: {text}")
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = Mouth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
