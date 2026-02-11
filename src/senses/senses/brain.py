import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import os

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.get_logger().info('🧠 Brain node has been started.')

        # Subscribe to speech input topic
        self.subscription = self.create_subscription(
            String,
            'speech_input',
            self.handle_input,
            10
        )

        # Publish to speech output topic
        self.publisher_ = self.create_publisher(
            String,
            'speech_output',
            10
        )

        # Get the Ollama API URL from env
        self.ollama_url = os.environ.get("OLLAMA_API_URL")
        if not self.ollama_url:
            self.get_logger().error("OLLAMA_API_URL environment variable not set. Falling back to default http://localhost:11434")
            self.ollama_url = "http://localhost:11434"  # fallback default

        # Choose which model to use (adjust as needed)
        self.model = "tinyllama"

    def handle_input(self, msg: String):
        prompt = msg.data.strip()
        if not prompt:
            self.get_logger().info("🤐 Received empty input. Ignoring.")
            return

        self.get_logger().info(f"📥 Prompt: {prompt}")

        SYSTEM_PROMPT = (
        "Robot voice assistant. Answer in ONE short sentence. "
        "Max 12 words. No lists, no examples, no disclaimers."
        )

        try:
            response = requests.post(
                f"{self.ollama_url}/api/generate",
                json={
                    "model": self.model,
                    "system": SYSTEM_PROMPT,
                    "prompt": prompt,
                    "stream": False,
                    "options": {
                        "num_predict": 40,   # hard max tokens generated
                        "temperature": 0.2,
                        "top_p": 0.9,
                    }
                },
                timeout=20                
            )

            if response.ok:
                result = response.json()
                reply = result.get("response", "").strip()
                self.get_logger().info(f"📤 Response: {reply}")

                msg = String()
                msg.data = reply
                self.publisher_.publish(msg)

            else:
                self.get_logger().error(f"❌ Ollama returned error {response.status_code}: {response.text}")

        except Exception as e:
            self.get_logger().error(f"❌ Error contacting Ollama: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
