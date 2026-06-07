"""Vision tools backed by a ROS camera frame and Amazon Bedrock."""

from __future__ import annotations

import threading
import time

import boto3
from sensor_msgs.msg import CompressedImage
from strands import tool


def _normalise_image_format(format_text: str) -> str:
    value = (format_text or "").lower()
    if "png" in value:
        return "png"
    if "webp" in value:
        return "webp"
    if "jpeg" in value or "jpg" in value or "compressed" in value:
        return "jpeg"
    return "jpeg"


def _extract_text(response: dict) -> str:
    content = (
        response.get("output", {})
        .get("message", {})
        .get("content", [])
    )
    parts = []
    for item in content:
        text = item.get("text")
        if text:
            parts.append(text)
    return "\n".join(parts).strip()


class VisionController:
    """Keeps the latest camera frame and exposes a Bedrock vision tool."""

    def __init__(
        self,
        ros_node,
        *,
        topic: str,
        model_id: str,
        aws_profile: str,
        aws_region: str,
        enabled: bool = True,
        frame_timeout_seconds: float = 3.0,
    ):
        self._node = ros_node
        self._topic = topic
        self._model_id = model_id
        self._aws_profile = aws_profile
        self._aws_region = aws_region
        self._enabled = enabled
        self._frame_timeout_seconds = frame_timeout_seconds
        self._condition = threading.Condition()
        self._latest_frame: CompressedImage | None = None
        self._latest_frame_time = 0.0

        self._subscription = ros_node.create_subscription(
            CompressedImage,
            topic,
            self._on_frame,
            1,
        )

    def _on_frame(self, msg: CompressedImage) -> None:
        with self._condition:
            self._latest_frame = msg
            self._latest_frame_time = time.monotonic()
            self._condition.notify_all()

    def _wait_for_frame(self) -> CompressedImage | None:
        deadline = time.monotonic() + self._frame_timeout_seconds
        with self._condition:
            while self._latest_frame is None and time.monotonic() < deadline:
                self._condition.wait(timeout=0.2)
            return self._latest_frame

    def _client(self):
        session = boto3.Session(
            profile_name=self._aws_profile or None,
            region_name=self._aws_region,
        )
        return session.client("bedrock-runtime")

    def make_tools(self) -> list:
        ctrl = self

        @tool
        def inspect_camera_view(question: str = "What can you see?") -> str:
            """Answer a visual question using the latest camera frame from the robot."""
            if not ctrl._enabled:
                return "ERROR: Vision tools are disabled."

            frame = ctrl._wait_for_frame()
            if frame is None:
                return f"ERROR: No camera frame received yet on {ctrl._topic}."

            image_bytes = bytes(frame.data)
            if not image_bytes:
                return f"ERROR: Latest camera frame on {ctrl._topic} was empty."

            image_format = _normalise_image_format(frame.format)
            prompt = (
                "You are looking through a robot camera. "
                "Answer the user's visual question briefly and concretely. "
                "Mention uncertainty when the image is unclear. "
                f"Question: {question}"
            )

            try:
                response = ctrl._client().converse(
                    modelId=ctrl._model_id,
                    messages=[
                        {
                            "role": "user",
                            "content": [
                                {"text": prompt},
                                {
                                    "image": {
                                        "format": image_format,
                                        "source": {"bytes": image_bytes},
                                    }
                                },
                            ],
                        }
                    ],
                    inferenceConfig={
                        "maxTokens": 300,
                        "temperature": 0.2,
                        "topP": 0.9,
                    },
                )
            except Exception as exc:
                ctrl._node.get_logger().warn(f"Vision Bedrock call failed: {type(exc).__name__}: {exc}")
                return f"ERROR: Vision request failed: {type(exc).__name__}: {exc}"

            text = _extract_text(response)
            if not text:
                return "ERROR: Vision model returned no text."
            return text

        return [inspect_camera_view]
