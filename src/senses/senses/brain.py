import os
import time
import threading
import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from strands import Agent, tool
from strands.models import BedrockModel
from strands_tools import calculator, current_time


# ----------------------------
# Custom tool example (optional)
# ----------------------------
@tool
def letter_counter(word: str, letter: str) -> int:
    """
    Count occurrences of a specific letter in a word.
    """
    if not isinstance(word, str) or not isinstance(letter, str):
        return 0
    if len(letter) != 1:
        raise ValueError("The 'letter' parameter must be a single character")
    return word.lower().count(letter.lower())


class Brain(Node):
    def __init__(self):
        super().__init__("brain")
        self.get_logger().info("🧠 Brain node has been started (Strands + Bedrock).")

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.subscription = self.create_subscription(
            String,
            "speech_input",
            self.handle_input,
            10,
        )

        self.publisher_ = self.create_publisher(
            String,
            "speech_output",
            10,
        )

        # ----------------------------
        # Params / env
        # ----------------------------
        self.declare_parameter("bedrock_region", "ap-southeast-2")
        self.declare_parameter("bedrock_model_id", "apac.anthropic.claude-3-haiku-20240307-v1:0")
        self.declare_parameter("temperature", 0.3)

        # You can also override via env if you prefer
        region = os.environ.get("AWS_REGION") or str(self.get_parameter("bedrock_region").value)
        model_id = os.environ.get("BEDROCK_MODEL_ID") or str(self.get_parameter("bedrock_model_id").value)
        temperature = float(os.environ.get("BEDROCK_TEMPERATURE") or self.get_parameter("temperature").value)

        # System prompt (keep short for voice)
        self.system_prompt = (
            "Robot voice assistant. Answer in ONE short sentence. "
            "Max 12 words. No lists, no examples, no disclaimers."
        )

        # ----------------------------
        # Strands logging (optional)
        # ----------------------------
        # If you want Strands debug logs, set STRANDS_LOG_LEVEL=DEBUG
        strands_level = os.environ.get("STRANDS_LOG_LEVEL", "INFO").upper()
        try:
            logging.getLogger("strands").setLevel(getattr(logging, strands_level, logging.INFO))
        except Exception:
            pass

        # ----------------------------
        # Create Bedrock model + agent
        # ----------------------------
        # IMPORTANT: For some models/regions you must use an inference profile id/arn.
        # Example: "apac.anthropic.claude-3-haiku-20240307-v1:0"
        self.bedrock_model = BedrockModel(
            model_id=model_id,
            region_name=region,
            temperature=temperature,
        )

        self.agent = Agent(
            model=self.bedrock_model,
            tools=[calculator, current_time, letter_counter],
        )

        self.get_logger().info(f"Using Bedrock region={region}, model_id={model_id}, temperature={temperature}")

        # Prevent overlapping requests (speech can arrive quickly)
        self._lock = threading.Lock()

    def handle_input(self, msg: String):
        prompt = (msg.data or "").strip()
        if not prompt:
            self.get_logger().info("🤐 Received empty input. Ignoring.")
            return

        # Run the call in a thread so we don't block the ROS executor
        threading.Thread(target=self._process_prompt, args=(prompt,), daemon=True).start()

    def _process_prompt(self, prompt: str):
        if not self._lock.acquire(blocking=False):
            self.get_logger().warn("Brain is busy; ignoring new prompt.")
            return

        try:
            self.get_logger().info(f"📥 Prompt: {prompt}")

            # Wrap prompt with system instructions (simple + effective)
            # This keeps your “voice assistant” constraints consistent.
            message = f"{self.system_prompt}\n\nUser: {prompt}"

            t0 = time.time()
            result = self.agent(message)
            t1 = time.time()

            # Strands Agent return types can vary by version.
            # Common cases:
            # - str
            # - object with .output / .text / dict-like content
            reply = self._extract_text(result).strip()

            if not reply:
                reply = "Sorry, I didn't catch that."

            self.get_logger().info(f"📤 Response ({(t1 - t0):.2f}s): {reply}")

            out = String()
            out.data = reply
            self.publisher_.publish(out)

        except Exception as e:
            self.get_logger().error(f"❌ Error calling Strands/Bedrock: {e}")
            out = String()
            out.data = "Sorry, I had a problem thinking."
            self.publisher_.publish(out)
        finally:
            self._lock.release()

    @staticmethod
    def _extract_text(result) -> str:
        """
        Make the Brain resilient to different Strands return shapes.
        """
        if result is None:
            return ""
        if isinstance(result, str):
            return result

        # dict-like
        if isinstance(result, dict):
            for k in ("response", "output", "text", "message", "content"):
                v = result.get(k)
                if isinstance(v, str) and v.strip():
                    return v
            return str(result)

        # object-like
        for attr in ("response", "output", "text", "message", "content"):
            if hasattr(result, attr):
                v = getattr(result, attr)
                if isinstance(v, str) and v.strip():
                    return v

        return str(result)


def main(args=None):
    rclpy.init(args=args)
    node = Brain()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
