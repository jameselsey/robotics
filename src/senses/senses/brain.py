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

from senses.movement_tools import MovementController


@tool
def letter_counter(word: str, letter: str) -> int:
    """Count occurrences of a specific letter in a word."""
    if not isinstance(word, str) or not isinstance(letter, str):
        return 0
    if len(letter) != 1:
        raise ValueError("The 'letter' parameter must be a single character")
    return word.lower().count(letter.lower())


class Brain(Node):
    def __init__(self):
        super().__init__("brain")
        self.get_logger().info("🧠 Brain node has been started (Strands + Bedrock).")

        self.create_subscription(String, "speech_input", self.handle_input, 10)
        self.publisher_ = self.create_publisher(String, "speech_output", 10)

        self._movement = MovementController(self)

        self.declare_parameter("bedrock_region", "ap-southeast-2")
        self.declare_parameter("bedrock_model_id", "apac.anthropic.claude-3-haiku-20240307-v1:0")
        self.declare_parameter("temperature", 0.3)

        region = os.environ.get("AWS_REGION") or str(self.get_parameter("bedrock_region").value)
        model_id = os.environ.get("BEDROCK_MODEL_ID") or str(self.get_parameter("bedrock_model_id").value)
        temperature = float(os.environ.get("BEDROCK_TEMPERATURE") or self.get_parameter("temperature").value)

        self.system_prompt = (
            "Robot voice assistant. For movement commands (move, drive, turn, stop, forward, backward, left, right), "
            "you MUST call the appropriate movement tool — do not just describe the action. "
            "For non-movement questions, answer in ONE short sentence. Max 12 words. No lists, no disclaimers."
        )

        strands_level = os.environ.get("STRANDS_LOG_LEVEL", "INFO").upper()
        try:
            logging.getLogger("strands").setLevel(getattr(logging, strands_level, logging.INFO))
        except Exception:
            pass

        self.bedrock_model = BedrockModel(model_id=model_id, region_name=region, temperature=temperature)
        self._tools = [calculator, current_time, letter_counter] + self._movement.make_tools()
        self.get_logger().info(f"Using Bedrock region={region}, model_id={model_id}, temperature={temperature}")
        self._lock = threading.Lock()

    def handle_input(self, msg: String):
        prompt = (msg.data or "").strip()
        if not prompt:
            return
        threading.Thread(target=self._process_prompt, args=(prompt,), daemon=True).start()

    def _process_prompt(self, prompt: str):
        if not self._lock.acquire(blocking=False):
            self.get_logger().warn("Brain is busy; ignoring new prompt.")
            return
        try:
            self.get_logger().info(f"📥 Prompt: {prompt}")
            message = f"{self.system_prompt}\n\nUser: {prompt}"

            agent = Agent(model=self.bedrock_model, tools=self._tools)

            t0 = time.time()
            result = agent(message)
            t1 = time.time()

            reply = self._extract_text(result).strip()
            if not reply:
                self.get_logger().info(f"📤 Response ({(t1 - t0):.2f}s): (action completed)")
                return

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
        if result is None:
            return ""
        if isinstance(result, str):
            return result
        if hasattr(result, "message"):
            msg = result.message
            if hasattr(msg, "content") and isinstance(msg.content, list):
                parts = [b.get("text", "") for b in msg.content if isinstance(b, dict) and b.get("text")]
                text = " ".join(parts).strip()
                if text:
                    return text
        if isinstance(result, dict):
            for k in ("response", "output", "text", "content"):
                v = result.get(k)
                if isinstance(v, str) and v.strip():
                    return v
        for attr in ("response", "output", "text", "content"):
            if hasattr(result, attr):
                v = getattr(result, attr)
                if isinstance(v, str) and v.strip():
                    return v
        return ""


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
