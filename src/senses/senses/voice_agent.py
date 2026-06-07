import asyncio
import base64
import json
import logging
import os
import queue
import select
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import boto3
import numpy as np
import pyaudio
import rclpy
from ament_index_python.packages import get_package_share_directory
from playsound import playsound
from rclpy.node import Node
from std_msgs.msg import String
from strands import tool
from strands.experimental.bidi import BidiAgent
from strands.experimental.bidi.models import BidiNovaSonicModel
from strands.experimental.bidi.types.events import (
    BidiAudioInputEvent,
    BidiAudioStreamEvent,
    BidiInterruptionEvent,
    BidiResponseStartEvent,
    BidiTextInputEvent,
)
from strands.experimental.bidi.types.io import BidiInput, BidiOutput
from strands_tools import calculator, current_time

from senses.movement_tools import MovementController
from senses.semantic_map_tools import SemanticMapController
from senses.vision_tools import VisionController

try:
    from gpiozero import PWMLED
except Exception:  # pragma: no cover - allows development away from robot GPIO
    PWMLED = None

try:
    from pywebrtc_audio import AudioProcessor
except Exception:  # pragma: no cover - lets the node run without AEC installed
    AudioProcessor = None


# ----------------------------
# Wyoming protocol helpers
# ----------------------------
def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Socket closed while receiving")
        buf += chunk
    return buf


def wyoming_send_event(
    sock: socket.socket,
    event_type: str,
    data: dict | None = None,
    payload: bytes = b"",
) -> None:
    if data is None:
        data = {}

    header = {
        "type": event_type,
        "data": data,
        "data_length": 0,
        "payload_length": len(payload),
    }
    line = (json.dumps(header, separators=(",", ":")) + "\n").encode("utf-8")
    sock.sendall(line)
    if payload:
        sock.sendall(payload)


def wyoming_recv_event(sock: socket.socket) -> tuple[str, dict, bytes]:
    line = b""
    while not line.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError("Socket closed while reading event header")
        line += chunk

    header = json.loads(line.decode("utf-8"))
    event_type = header.get("type")
    data = header.get("data") or {}

    data_length = int(header.get("data_length") or 0)
    payload_length = int(header.get("payload_length") or 0)

    if data_length > 0:
        _ = _recv_exact(sock, data_length)

    payload = b""
    if payload_length > 0:
        payload = _recv_exact(sock, payload_length)

    return event_type, data, payload


class LedController:
    def __init__(self, node: Node, pin: int, pwm_hz: int, amp_gain: float, smooth_alpha: float):
        self._node = node
        self._amp_gain = amp_gain
        self._smooth_alpha = smooth_alpha
        self._brightness = 0.0
        self._lock = threading.Lock()
        self._led = None

        if PWMLED is None:
            node.get_logger().warn("gpiozero is not available; LED feedback is disabled.")
            return

        try:
            self._led = PWMLED(pin, frequency=pwm_hz)
            self.off()
        except Exception as exc:
            node.get_logger().warn(f"Could not initialize LED on GPIO {pin}: {exc}")
            self._led = None

    def wake(self) -> None:
        self.set(0.65)

    def off(self) -> None:
        self.set(0.0)

    def set(self, value: float) -> None:
        if self._led is None:
            return
        with self._lock:
            self._brightness = max(0.0, min(1.0, value))
            try:
                self._led.value = self._brightness
            except Exception:
                self._led = None

    def audio_bytes(self, data: bytes) -> None:
        if self._led is None:
            return
        if not data:
            self.set(0.0)
            return
        audio = np.frombuffer(data, dtype=np.int16)
        if audio.size == 0:
            self.set(0.0)
            return
        amp = float(np.sqrt(np.maximum(1e-12, np.mean((audio.astype(np.float32) / 32768.0) ** 2))))
        target = min(1.0, self._amp_gain * amp)
        with self._lock:
            brightness = self._smooth_alpha * self._brightness + (1.0 - self._smooth_alpha) * target
            self._brightness = max(0.0, min(1.0, brightness))
            try:
                self._led.value = self._brightness
            except Exception:
                self._led = None

    def close(self) -> None:
        try:
            self.off()
            if self._led is not None:
                self._led.close()
        except Exception:
            pass


class AudioBuffer:
    """Small byte buffer used by the PyAudio playback callback."""
    def __init__(self, size: int | None = None):
        self._size = size or 0

    def start(self) -> None:
        self._buffer = queue.Queue(self._size)
        self._data = bytearray()

    def stop(self) -> None:
        if hasattr(self, "_data"):
            self._data.clear()
        if hasattr(self, "_buffer"):
            self._buffer.put_nowait(b"")
            self._buffer = queue.Queue(self._size)

    def put(self, chunk: bytes) -> None:
        if self._buffer.full():
            try:
                self._buffer.get_nowait()
            except queue.Empty:
                pass
        self._buffer.put_nowait(chunk)

    def get(self, byte_count: int | None = None) -> bytes:
        if not byte_count:
            self._data.extend(self._buffer.get())
            byte_count = len(self._data)

        while len(self._data) < byte_count:
            try:
                self._data.extend(self._buffer.get_nowait())
            except queue.Empty:
                break

        self._data.extend(b"\x00" * max(byte_count - len(self._data), 0))
        data = self._data[:byte_count]
        del self._data[:byte_count]
        return bytes(data)

    def clear(self) -> None:
        if hasattr(self, "_data"):
            self._data.clear()
        while True:
            try:
                self._buffer.get_nowait()
            except queue.Empty:
                break


class ActivityTracker:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_activity = time.monotonic()

    def mark(self) -> None:
        with self._lock:
            self._last_activity = time.monotonic()

    def idle_seconds(self) -> float:
        with self._lock:
            return time.monotonic() - self._last_activity


class PlaybackState:
    """Tracks recent speaker activity so the mic can be muted while the robot speaks."""
    def __init__(self):
        self._lock = threading.Lock()
        self._last_output_audio = 0.0

    def mark_output_audio(self) -> None:
        with self._lock:
            self._last_output_audio = time.monotonic()

    def seconds_since_output_audio(self) -> float:
        with self._lock:
            if self._last_output_audio <= 0.0:
                return 999999.0
            return time.monotonic() - self._last_output_audio


class FarEndReferenceBuffer:
    """Speaker audio reference used by WebRTC AEC to remove robot self-audio from the mic."""
    def __init__(self, max_chunks: int = 100):
        self._queue: queue.Queue[np.ndarray] = queue.Queue(max_chunks)
        self._carry = np.array([], dtype=np.int16)

    def put(self, samples: np.ndarray) -> None:
        if samples.size == 0:
            return
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
        self._queue.put_nowait(samples.astype(np.int16, copy=True))

    def read(self, sample_count: int) -> np.ndarray:
        chunks = []
        if self._carry.size:
            chunks.append(self._carry)
            self._carry = np.array([], dtype=np.int16)

        total = sum(chunk.size for chunk in chunks)
        while total < sample_count:
            try:
                chunk = self._queue.get_nowait()
            except queue.Empty:
                break
            chunks.append(chunk)
            total += chunk.size

        if chunks:
            data = np.concatenate(chunks)
        else:
            data = np.array([], dtype=np.int16)

        if data.size >= sample_count:
            out = data[:sample_count]
            self._carry = data[sample_count:]
            return out.astype(np.int16, copy=False)

        out = np.zeros(sample_count, dtype=np.int16)
        if data.size:
            out[:data.size] = data
        return out

    def clear(self) -> None:
        self._carry = np.array([], dtype=np.int16)
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break


class DirectAudioInput:
    """Reads microphone frames, applies WebRTC audio processing, and sends them to Nova."""
    def __init__(
        self,
        node: Node,
        activity: ActivityTracker,
        input_device_index: int | None,
        frames_per_buffer: int,
        threshold: float,
        silence_gate_threshold: float,
        silence_gate_enabled: bool,
        speech_gate_threshold: float,
        playback_state: PlaybackState,
        mute_during_output_seconds: float,
        audio_processor,
        far_end_buffer: FarEndReferenceBuffer | None,
    ):
        self._node = node
        self._activity = activity
        self._input_device_index = input_device_index
        self._frames_per_buffer = frames_per_buffer
        self._threshold = threshold
        self._silence_gate_threshold = silence_gate_threshold
        self._silence_gate_enabled = silence_gate_enabled
        self._speech_gate_threshold = speech_gate_threshold
        self._playback_state = playback_state
        self._mute_during_output_seconds = mute_during_output_seconds
        self._audio_processor = audio_processor
        self._far_end_buffer = far_end_buffer
        self.chunk_count = 0
        self.active_chunk_count = 0
        self.gated_chunk_count = 0
        self.output_muted_chunk_count = 0
        self.aec_processed_chunk_count = 0
        self.speech_probability = 0.0
        self.last_amp = 0.0
        self._last_log = 0.0

    async def start(self, agent) -> None:
        self._channels = agent.model.config["audio"]["channels"]
        self._format = agent.model.config["audio"]["format"]
        self._rate = agent.model.config["audio"]["input_rate"]
        self._node.get_logger().info(
            f"Opening direct Nova microphone stream rate={self._rate}, channels={self._channels}, "
            f"device={self._input_device_index}, frames={self._frames_per_buffer}"
        )
        self._audio = pyaudio.PyAudio()
        self._stream = self._audio.open(
            format=pyaudio.paInt16,
            channels=self._channels,
            rate=self._rate,
            input=True,
            frames_per_buffer=self._frames_per_buffer,
            input_device_index=self._input_device_index,
        )
        self._node.get_logger().info("Direct Nova microphone stream is open.")

    async def stop(self) -> None:
        self._node.get_logger().info("Stopping direct Nova microphone stream...")
        if hasattr(self, "_stream"):
            try:
                if self._stream.is_active():
                    self._stream.stop_stream()
            except OSError as exc:
                self._node.get_logger().debug(f"Direct Nova microphone stream already stopped: {exc}")
            try:
                self._stream.close()
            except OSError as exc:
                self._node.get_logger().debug(f"Direct Nova microphone stream close ignored: {exc}")
        if hasattr(self, "_audio"):
            self._audio.terminate()

    async def __call__(self):
        data = await asyncio.to_thread(
            self._stream.read,
            self._frames_per_buffer,
            exception_on_overflow=False,
        )
        self.chunk_count += 1
        audio = np.frombuffer(data, dtype=np.int16)
        if audio.size:
            # AEC needs both sides: near-end mic audio plus far-end audio currently played by the speaker.
            if self._audio_processor is not None and self._far_end_buffer is not None:
                far = self._far_end_buffer.read(audio.size)
                audio = self._audio_processor.process(audio, far)
                data = audio.astype(np.int16, copy=False).tobytes()
                self.aec_processed_chunk_count += 1
                try:
                    self.speech_probability = float(self._audio_processor.speech_probability)
                except Exception:
                    self.speech_probability = 0.0

            amp = float(np.abs(audio).mean())
            self.last_amp = amp
            mute_for_output = (
                self._mute_during_output_seconds > 0.0
                and self._playback_state.seconds_since_output_audio() < self._mute_during_output_seconds
            )
            if mute_for_output:
                # Barge-in is intentionally disabled for now: do not send live mic audio while speaking.
                self.output_muted_chunk_count += 1
                data = b"\x00" * len(data)
            else:
                if amp >= self._threshold:
                    self.active_chunk_count += 1
                    self._activity.mark()
                speech_detected = (
                    self._audio_processor is not None
                    and self.speech_probability >= self._speech_gate_threshold
                )
                # Preserve low-amplitude frames when WebRTC still thinks they contain speech.
                if self._silence_gate_enabled and amp < self._silence_gate_threshold and not speech_detected:
                    self.gated_chunk_count += 1
                    data = b"\x00" * len(data)
            now = time.monotonic()
            if now - self._last_log >= 2.0:
                self._last_log = now
                self._node.get_logger().debug(
                    f"Nova input chunks={self.chunk_count}, active={self.active_chunk_count}, "
                    f"gated={self.gated_chunk_count}, output_muted={self.output_muted_chunk_count}, "
                    f"aec={self.aec_processed_chunk_count}, speech_prob={self.speech_probability:.2f}, "
                    f"last_amp={self.last_amp:.1f}, threshold={self._threshold:.1f}, "
                    f"gate={self._silence_gate_threshold:.1f}"
                )
        return BidiAudioInputEvent(
            audio=base64.b64encode(data).decode("utf-8"),
            channels=self._channels,
            format=self._format,
            sample_rate=self._rate,
        )


class LedAudioOutput:
    """Plays Nova audio, drives the LED, and feeds played samples back into the AEC reference."""
    def __init__(
        self,
        led: LedController,
        activity: ActivityTracker,
        node: Node,
        playback_state: PlaybackState,
        far_end_buffer: FarEndReferenceBuffer | None = None,
        audio_processor=None,
        output_device_index: int | None = None,
        output_buffer_size: int | None = None,
        output_frames_per_buffer: int = 1024,
        prebuffer_chunks: int = 3,
    ):
        self._led = led
        self._activity = activity
        self._node = node
        self._playback_state = playback_state
        self._far_end_buffer = far_end_buffer
        self._audio_processor = audio_processor
        self._device_index = output_device_index
        self._frames_per_buffer = output_frames_per_buffer
        self._prebuffer_chunks = max(0, prebuffer_chunks)
        self._playback_started = self._prebuffer_chunks == 0
        self._drop_audio_until_next_response = False
        self._response_chunk_count = 0
        self._dropped_after_interrupt_count = 0
        self._buffer = AudioBuffer(output_buffer_size)
        self.output_chunk_count = 0
        self.playback_callback_count = 0
        self.last_output_bytes = 0
        self._last_log = 0.0

    async def start(self, agent) -> None:
        self._channels = agent.model.config["audio"]["channels"]
        self._rate = agent.model.config["audio"]["output_rate"]
        self._node.get_logger().info(
            f"Opening Nova Sonic speaker output stream rate={self._rate}, "
            f"channels={self._channels}, device={self._device_index}, prebuffer_chunks={self._prebuffer_chunks}"
        )
        self._buffer.start()
        self._audio = pyaudio.PyAudio()
        try:
            self._stream = self._audio.open(
                channels=self._channels,
                format=pyaudio.paInt16,
                frames_per_buffer=self._frames_per_buffer,
                output=True,
                output_device_index=self._device_index,
                rate=self._rate,
                stream_callback=self._callback,
            )
        except OSError as exc:
            if self._device_index is None:
                raise
            self._node.get_logger().warn(
                f"Could not open configured output device {self._device_index}: {exc}. "
                "Falling back to default output device."
            )
            self._device_index = None
            self._stream = self._audio.open(
                channels=self._channels,
                format=pyaudio.paInt16,
                frames_per_buffer=self._frames_per_buffer,
                output=True,
                output_device_index=None,
                rate=self._rate,
                stream_callback=self._callback,
            )
        self._node.get_logger().info(f"Nova Sonic speaker output stream is open on device={self._device_index}.")

    async def stop(self) -> None:
        self._node.get_logger().info("Stopping Nova Sonic speaker output stream...")
        if hasattr(self, "_stream"):
            try:
                if self._stream.is_active():
                    self._stream.stop_stream()
            except OSError as exc:
                self._node.get_logger().debug(f"Nova Sonic speaker stream already stopped: {exc}")
            try:
                self._stream.close()
            except OSError as exc:
                self._node.get_logger().debug(f"Nova Sonic speaker stream close ignored: {exc}")
        if hasattr(self, "_audio"):
            self._audio.terminate()
        self._buffer.stop()
        self._led.off()

    async def __call__(self, event) -> None:
        if isinstance(event, BidiResponseStartEvent):
            if self._drop_audio_until_next_response:
                self._node.get_logger().info(
                    f"Nova response start after interruption cleanup; dropped {self._dropped_after_interrupt_count} stale audio chunks."
                )
                self._playback_started = self._prebuffer_chunks == 0
            self._drop_audio_until_next_response = False
            self._dropped_after_interrupt_count = 0
            self._response_chunk_count = 0
        elif isinstance(event, BidiAudioStreamEvent):
            data = base64.b64decode(event["audio"])
            self.output_chunk_count += 1
            self.last_output_bytes = len(data)
            if self._drop_audio_until_next_response:
                self._dropped_after_interrupt_count += 1
                return
            self._response_chunk_count += 1
            self._buffer.put(data)
            if not self._playback_started and self._response_chunk_count >= self._prebuffer_chunks:
                self._playback_started = True
                self._node.get_logger().info(
                    f"Nova speaker prebuffer ready after {self._response_chunk_count} chunks."
                )
            self._activity.mark()
            self._playback_state.mark_output_audio()
            now = time.monotonic()
            if now - self._last_log >= 2.0:
                self._last_log = now
                self._node.get_logger().debug(
                    f"Nova output chunks={self.output_chunk_count}, "
                    f"last_bytes={self.last_output_bytes}"
                )
        elif isinstance(event, BidiInterruptionEvent):
            # Barge-in is disabled locally, but keep this cleanup in case Nova still emits an interruption.
            self._interrupt_playback("Nova interruption event received", drop_until_next_response=True)

    def _interrupt_playback(self, reason: str, drop_until_next_response: bool) -> None:
        action = "clearing speaker buffer and dropping stale audio" if drop_until_next_response else "clearing speaker buffer"
        self._node.get_logger().info(f"{reason}; {action}.")
        self._drop_audio_until_next_response = drop_until_next_response
        self._response_chunk_count = 0
        self._playback_started = False
        self._buffer.clear()
        if self._far_end_buffer is not None:
            self._far_end_buffer.clear()
        if self._audio_processor is not None:
            try:
                self._audio_processor.reset()
            except Exception as exc:
                self._node.get_logger().debug(f"WebRTC audio processor reset ignored: {exc}")
        self._led.wake()

    def _callback(self, _in_data, frame_count: int, *_args):
        self.playback_callback_count += 1
        byte_count = frame_count * self._channels * pyaudio.get_sample_size(pyaudio.paInt16)
        if not self._playback_started:
            data = b"\x00" * byte_count
        else:
            data = self._buffer.get(byte_count)
        # Feed exactly what we hand to the speaker into the far-end reference for echo cancellation.
        if self._far_end_buffer is not None and data:
            self._far_end_buffer.put(np.frombuffer(data, dtype=np.int16))
        self._led.audio_bytes(data)
        return (data, pyaudio.paContinue)


def _optional_device_index(value: int) -> int | None:
    return value if value >= 0 else None


def _param_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class VoiceAgent(Node):
    def __init__(self):
        super().__init__("voice_agent")
        self.get_logger().info("Voice agent node started (wake word + Nova Sonic bidi).")

        self.state_pub = self.create_publisher(String, "voice_state", 10)
        self._movement = MovementController(self)

        self.declare_parameter("oww_host", "127.0.0.1")
        self.declare_parameter("oww_port", 10400)
        self.declare_parameter("wake_model_name", "computer")
        self.declare_parameter("chunk_size", 1280)
        self.declare_parameter("input_device_name", "Brio")
        self.declare_parameter("input_device_index", -1)
        self.declare_parameter("output_device_index", -1)
        self.declare_parameter("wake_ack_delay", 0.25)

        self.declare_parameter("aws_profile", os.environ.get("AWS_PROFILE", "default"))
        self.declare_parameter("aws_region", os.environ.get("AWS_REGION") or os.environ.get("AWS_DEFAULT_REGION", "us-east-1"))
        self.declare_parameter("nova_model_id", os.environ.get("NOVA_SONIC_MODEL_ID", "amazon.nova-2-sonic-v1:0"))
        self.declare_parameter("nova_voice", os.environ.get("NOVA_SONIC_VOICE", "amy"))
        self.declare_parameter("nova_output_rate", int(os.environ.get("NOVA_SONIC_OUTPUT_RATE", "16000")))
        self.declare_parameter("endpointing_sensitivity", os.environ.get("NOVA_SONIC_ENDPOINTING_SENSITIVITY", "LOW"))
        self.declare_parameter("temperature", float(os.environ.get("NOVA_SONIC_TEMPERATURE", "0.7")))
        self.declare_parameter("top_p", float(os.environ.get("NOVA_SONIC_TOP_P", "0.9")))
        self.declare_parameter("max_tokens", int(os.environ.get("NOVA_SONIC_MAX_TOKENS", "1024")))
        self.declare_parameter("idle_timeout_seconds", 45.0)
        self.declare_parameter("max_session_seconds", 420.0)
        self.declare_parameter("audio_activity_threshold", 250.0)
        self.declare_parameter("audio_silence_gate_enabled", True)
        self.declare_parameter("audio_silence_gate_threshold", 650.0)
        self.declare_parameter("audio_speech_gate_threshold", 0.6)
        self.declare_parameter("audio_processing_enabled", True)
        self.declare_parameter("audio_processing_stream_delay_ms", -1)
        self.declare_parameter("debug_text_probe", os.environ.get("NOVA_SONIC_DEBUG_TEXT_PROBE", ""))
        self.declare_parameter("session_heartbeat_seconds", 5.0)
        self.declare_parameter("strands_log_level", os.environ.get("STRANDS_LOG_LEVEL", "DEBUG"))
        self.declare_parameter("input_frames_per_buffer", 160)
        self.declare_parameter("output_frames_per_buffer", 160)
        self.declare_parameter("output_prebuffer_chunks", 0)
        self.declare_parameter("mute_input_during_output_seconds", 0.75)
        self.declare_parameter("rooms_config_path", "")
        self.declare_parameter("vision_enabled", True)
        self.declare_parameter("vision_topic", os.environ.get("VISION_TOPIC", "/image_viz/compressed"))
        self.declare_parameter("vision_model_id", os.environ.get("VISION_MODEL_ID", "amazon.nova-lite-v1:0"))
        self.declare_parameter("vision_frame_timeout_seconds", 3.0)

        self.declare_parameter("led_pin", 19)
        self.declare_parameter("led_pwm_hz", 400)
        self.declare_parameter("led_amp_gain", 3.0)
        self.declare_parameter("led_smooth_alpha", 0.25)

        self.oww_host = str(self.get_parameter("oww_host").value)
        self.oww_port = int(self.get_parameter("oww_port").value)
        self.wake_model_name = str(self.get_parameter("wake_model_name").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.input_device_name = str(self.get_parameter("input_device_name").value)
        self.input_device_index = int(self.get_parameter("input_device_index").value)
        self.output_device_index = int(self.get_parameter("output_device_index").value)
        self.wake_ack_delay = float(self.get_parameter("wake_ack_delay").value)

        self.aws_profile = str(self.get_parameter("aws_profile").value)
        self.aws_region = str(self.get_parameter("aws_region").value)
        self.nova_model_id = str(self.get_parameter("nova_model_id").value)
        self.nova_voice = str(self.get_parameter("nova_voice").value)
        self.nova_output_rate = int(self.get_parameter("nova_output_rate").value)
        self.endpointing_sensitivity = str(self.get_parameter("endpointing_sensitivity").value).upper()
        self.temperature = float(self.get_parameter("temperature").value)
        self.top_p = float(self.get_parameter("top_p").value)
        self.max_tokens = int(self.get_parameter("max_tokens").value)
        self.idle_timeout_seconds = float(self.get_parameter("idle_timeout_seconds").value)
        self.max_session_seconds = float(self.get_parameter("max_session_seconds").value)
        self.audio_activity_threshold = float(self.get_parameter("audio_activity_threshold").value)
        self.audio_silence_gate_enabled = _param_bool(self.get_parameter("audio_silence_gate_enabled").value)
        self.audio_silence_gate_threshold = float(self.get_parameter("audio_silence_gate_threshold").value)
        self.audio_speech_gate_threshold = float(self.get_parameter("audio_speech_gate_threshold").value)
        self.audio_processing_enabled = _param_bool(self.get_parameter("audio_processing_enabled").value)
        self.debug_text_probe = str(self.get_parameter("debug_text_probe").value).strip()
        self.session_heartbeat_seconds = float(self.get_parameter("session_heartbeat_seconds").value)
        self.strands_log_level = str(self.get_parameter("strands_log_level").value).upper()
        self.input_frames_per_buffer = int(self.get_parameter("input_frames_per_buffer").value)
        self.output_frames_per_buffer = int(self.get_parameter("output_frames_per_buffer").value)
        self.audio_processing_stream_delay_ms = int(self.get_parameter("audio_processing_stream_delay_ms").value)
        if self.audio_processing_stream_delay_ms < 0:
            self.audio_processing_stream_delay_ms = int(self.output_frames_per_buffer / 16000 * 1000)
        self.output_prebuffer_chunks = int(self.get_parameter("output_prebuffer_chunks").value)
        self.mute_input_during_output_seconds = float(self.get_parameter("mute_input_during_output_seconds").value)
        self.rooms_config_path = str(self.get_parameter("rooms_config_path").value).strip()
        self.vision_enabled = _param_bool(self.get_parameter("vision_enabled").value)
        self.vision_topic = str(self.get_parameter("vision_topic").value).strip()
        self.vision_model_id = str(self.get_parameter("vision_model_id").value).strip()
        self.vision_frame_timeout_seconds = float(self.get_parameter("vision_frame_timeout_seconds").value)

        self.RATE = 16000
        self.CHANNELS = 1
        self.WIDTH_BYTES = 2
        self.FORMAT = pyaudio.paInt16

        package_dir = Path(get_package_share_directory("senses"))
        self.sound_path = str(package_dir / "resource" / "r2-sound-acknowledged.mp3")
        self.stop_sound_path = str(package_dir / "resource" / "stop-listening.mp3")
        if not self.rooms_config_path:
            self.rooms_config_path = str(package_dir / "config" / "rooms.yaml")
        self._semantic_map = SemanticMapController(self, self.rooms_config_path)
        self._vision = VisionController(
            self,
            topic=self.vision_topic,
            model_id=self.vision_model_id,
            aws_profile=self.aws_profile,
            aws_region=self.aws_region,
            enabled=self.vision_enabled,
            frame_timeout_seconds=self.vision_frame_timeout_seconds,
        )

        try:
            logging.getLogger("strands").setLevel(getattr(logging, self.strands_log_level, logging.DEBUG))
            logging.getLogger("aws_sdk_bedrock_runtime").setLevel(logging.DEBUG)
            logging.getLogger("smithy").setLevel(logging.DEBUG)
        except Exception:
            pass

        self.pa = pyaudio.PyAudio()
        self._log_audio_devices()
        self.input_device = self._find_audio_device(self.input_device_name, self.input_device_index)
        self.output_device = _optional_device_index(self.output_device_index)
        self.led = LedController(
            self,
            pin=int(self.get_parameter("led_pin").value),
            pwm_hz=int(self.get_parameter("led_pwm_hz").value),
            amp_gain=float(self.get_parameter("led_amp_gain").value),
            smooth_alpha=float(self.get_parameter("led_smooth_alpha").value),
        )

        self._stop = False
        self._thread = threading.Thread(target=self._wake_loop, daemon=True)
        self._thread.start()

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Voice state: {state}")

    def _log_audio_devices(self) -> None:
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            name = info.get("name", "")
            max_input = int(info.get("maxInputChannels", 0))
            max_output = int(info.get("maxOutputChannels", 0))
            if max_input > 0 or max_output > 0:
                self.get_logger().info(
                    f"Audio device index={i}, inputs={max_input}, outputs={max_output}, name={name}"
                )

    def _find_audio_device(self, device_name: str, fallback_index: int) -> int | None:
        self.get_logger().info(f"Searching for audio input device containing '{device_name}'...")
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            name = info.get("name", "")
            if info.get("maxInputChannels", 0) > 0 and device_name.lower() in name.lower():
                self.get_logger().info(f"Found audio input device: {name} (index={i})")
                return i
        if fallback_index >= 0:
            self.get_logger().warn(f"Device '{device_name}' not found, using fallback index {fallback_index}")
            return fallback_index
        self.get_logger().warn(f"Device '{device_name}' not found, using system default input device")
        return None

    def _play_sound_with_led(self, sound_path: str, label: str) -> None:
        """Play an MP3 and drive the LED from the decoded audio amplitude."""
        try:
            playsound(sound_path, block=False)
        except Exception as exc:
            self.get_logger().warn(f"Could not play {label} sound: {exc}")
            return

        def follow_audio() -> None:
            cmd = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-i",
                sound_path,
                "-f",
                "s16le",
                "-acodec",
                "pcm_s16le",
                "-ac",
                "1",
                "-ar",
                "16000",
                "pipe:1",
            ]
            bytes_per_second = 16000 * 2
            chunk_bytes = int(bytes_per_second * 0.02)
            try:
                with subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL) as proc:
                    if proc.stdout is None:
                        return
                    while True:
                        chunk = proc.stdout.read(chunk_bytes)
                        if not chunk:
                            break
                        self.led.audio_bytes(chunk)
                        time.sleep(len(chunk) / bytes_per_second)
            except Exception as exc:
                self.get_logger().debug(f"Could not drive LED from {label} sound: {exc}")
            finally:
                self.led.off()

        threading.Thread(target=follow_audio, daemon=True).start()

    def destroy_node(self):
        self._stop = True
        self.led.close()
        try:
            if getattr(self, "pa", None):
                self.pa.terminate()
        except Exception:
            pass
        super().destroy_node()

    def _wake_loop(self):
        self._publish_state("idle")
        while rclpy.ok() and not self._stop:
            sock = None
            mic_stream = None
            try:
                sock = socket.create_connection((self.oww_host, self.oww_port), timeout=5)
                sock.settimeout(None)
                wyoming_send_event(sock, "detect", {"names": [self.wake_model_name]})
                wyoming_send_event(sock, "audio-start", {"rate": self.RATE, "width": self.WIDTH_BYTES, "channels": self.CHANNELS})

                mic_stream = self.pa.open(
                    format=self.FORMAT,
                    channels=self.CHANNELS,
                    rate=self.RATE,
                    input=True,
                    frames_per_buffer=self.chunk_size,
                    input_device_index=self.input_device,
                )

                self.get_logger().info(f"Listening for wake word '{self.wake_model_name}'...")
                detected = False
                detected_name = None

                while rclpy.ok() and not self._stop:
                    pcm_bytes = mic_stream.read(self.chunk_size, exception_on_overflow=False)
                    wyoming_send_event(
                        sock,
                        "audio-chunk",
                        {"rate": self.RATE, "width": self.WIDTH_BYTES, "channels": self.CHANNELS},
                        payload=pcm_bytes,
                    )

                    ready, _, _ = select.select([sock], [], [], 0)
                    if not ready:
                        continue

                    event_type, data, _payload = wyoming_recv_event(sock)
                    if event_type == "detection":
                        detected = True
                        detected_name = data.get("name") or self.wake_model_name
                        break
                    if event_type == "not-detected":
                        break

                try:
                    wyoming_send_event(sock, "audio-stop", {})
                except Exception:
                    pass
                if mic_stream:
                    mic_stream.stop_stream()
                    mic_stream.close()
                    mic_stream = None

                if not detected:
                    continue

                # Close the wake-word mic stream before Nova opens its own low-latency stream.
                self.get_logger().info(f"Wake word detected (model={detected_name})")
                self._publish_state("wake_detected")
                self._play_sound_with_led(self.sound_path, "wake")
                time.sleep(self.wake_ack_delay)

                conversation_reason = asyncio.run(self._run_conversation())
                if conversation_reason == "stop requested":
                    self._play_sound_with_led(self.stop_sound_path, "stop")
                self.led.off()
                self._publish_state("idle")

            except Exception as exc:
                self.get_logger().error(f"Voice wake loop error: {exc}")
                self.led.off()
                time.sleep(1.0)
            finally:
                try:
                    if mic_stream:
                        mic_stream.close()
                except Exception:
                    pass
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass

    def _provider_config(self) -> dict[str, Any]:
        config: dict[str, Any] = {
            "audio": {"voice": self.nova_voice, "output_rate": self.nova_output_rate},
            "inference": {
                "max_tokens": self.max_tokens,
                "top_p": self.top_p,
                "temperature": self.temperature,
            },
        }
        if self.endpointing_sensitivity and "nova-2" in self.nova_model_id:
            config["turn_detection"] = {"endpointingSensitivity": self.endpointing_sensitivity}
        return config

    def _make_sleep_tool(self, stop_event: threading.Event):
        @tool
        def go_to_sleep() -> str:
            """End this voice conversation and return the robot to wake-word listening mode."""
            stop_event.set()
            return "Going back to sleep."

        return go_to_sleep

    async def _idle_watch(self, activity: ActivityTracker, stop_event: threading.Event) -> str:
        while not stop_event.is_set():
            await asyncio.sleep(1.0)
            if self.idle_timeout_seconds > 0 and activity.idle_seconds() >= self.idle_timeout_seconds:
                stop_event.set()
                return "idle timeout"
        return "stop requested"

    async def _session_heartbeat(
        self,
        activity: ActivityTracker,
        stop_event: threading.Event,
        audio_input: DirectAudioInput,
        audio_output: LedAudioOutput,
    ) -> str:
        while not stop_event.is_set():
            await asyncio.sleep(max(1.0, self.session_heartbeat_seconds))
            self.get_logger().debug(
                f"Nova heartbeat idle={activity.idle_seconds():.1f}s, "
                f"input_chunks={audio_input.chunk_count}, active_input={audio_input.active_chunk_count}, "
                f"gated_input={audio_input.gated_chunk_count}, output_muted={audio_input.output_muted_chunk_count}, "
                f"aec={audio_input.aec_processed_chunk_count}, speech_prob={audio_input.speech_probability:.2f}, "
                f"last_amp={audio_input.last_amp:.1f}, output_chunks={audio_output.output_chunk_count}, "
                f"playback_callbacks={audio_output.playback_callback_count}"
            )
        return "heartbeat stopped"

    async def _run_agent_loop(self, agent: BidiAgent, inputs: list[BidiInput], outputs: list[BidiOutput]) -> None:
        stopping_io = threading.Event()

        async def start_io() -> None:
            for io in [*inputs, *outputs]:
                start = getattr(io, "start", None)
                if start is not None:
                    await start(agent)

        async def stop_io() -> None:
            for io in [*inputs, *outputs]:
                stop = getattr(io, "stop", None)
                if stop is not None:
                    try:
                        await stop()
                    except Exception as exc:
                        self.get_logger().warn(f"IO stop failed for {type(io).__name__}: {type(exc).__name__}: {exc}")

        async def run_inputs() -> None:
            try:
                while True:
                    for input_ in inputs:
                        event = await input_()
                        await agent.send(event)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if stopping_io.is_set():
                    self.get_logger().debug(f"Nova input/send loop stopped during shutdown: {type(exc).__name__}: {exc}")
                    return
                self.get_logger().error(f"Nova input/send loop failed: {type(exc).__name__}: {exc}")
                raise

        async def run_outputs(inputs_task: asyncio.Task) -> None:
            try:
                async for event in agent.receive():
                    event_type = event.get("type", type(event).__name__) if isinstance(event, dict) else type(event).__name__
                    self.get_logger().debug(f"Nova event received: {event_type}")
                    await asyncio.gather(*[output(event) for output in outputs])
                self.get_logger().warn("Nova receive loop ended without an exception.")
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.get_logger().error(f"Nova receive loop failed: {type(exc).__name__}: {exc}")
                raise
            finally:
                inputs_task.cancel()

        self.get_logger().info("Starting explicit Nova Sonic agent loop...")
        inputs_task: asyncio.Task | None = None
        outputs_task: asyncio.Task | None = None
        try:
            await agent.start()
            self.get_logger().info("Nova Sonic agent.start() completed.")
            await start_io()
            inputs_task = asyncio.create_task(run_inputs())
            outputs_task = asyncio.create_task(run_outputs(inputs_task))
            done, pending = await asyncio.wait({inputs_task, outputs_task}, return_when=asyncio.FIRST_EXCEPTION)
            for task in done:
                if task.cancelled():
                    continue
                exc = task.exception()
                if exc is not None:
                    raise exc
            self.get_logger().warn("Nova agent IO tasks ended cleanly; this usually means the model stream closed.")
        finally:
            stopping_io.set()
            for task in (inputs_task, outputs_task):
                if task is not None and not task.done():
                    task.cancel()
            await asyncio.gather(
                *[task for task in (inputs_task, outputs_task) if task is not None],
                return_exceptions=True,
            )
            try:
                await asyncio.wait_for(agent.stop(), timeout=8.0)
                self.get_logger().info("Nova Sonic agent.stop() completed.")
            except asyncio.TimeoutError:
                self.get_logger().warn("Timed out waiting for Nova Sonic agent.stop().")
            except Exception as exc:
                self.get_logger().warn(f"Nova Sonic agent.stop() failed: {type(exc).__name__}: {exc}")
            finally:
                await stop_io()

    async def _run_conversation(self):
        self._publish_state("conversation")
        stop_event = threading.Event()
        activity = ActivityTracker()
        playback_state = PlaybackState()
        far_end_buffer = FarEndReferenceBuffer()
        audio_processor = None
        if self.audio_processing_enabled:
            if AudioProcessor is None:
                self.get_logger().warn("pywebrtc-audio is not installed; AEC/noise suppression disabled.")
            else:
                # pywebrtc-audio wraps WebRTC AEC/NS/AGC. It expects 10 ms mono PCM frames at 16 kHz.
                audio_processor = AudioProcessor(
                    sample_rate=16000,
                    num_channels=1,
                    echo_cancellation=True,
                    noise_suppression=True,
                    auto_gain_control=True,
                    stream_delay_ms=self.audio_processing_stream_delay_ms,
                )
                self.get_logger().info(
                    f"WebRTC audio processing enabled: AEC+NS+AGC, "
                    f"stream_delay_ms={self.audio_processing_stream_delay_ms}"
                )

        boto_session = boto3.Session(profile_name=self.aws_profile or None, region_name=self.aws_region)
        model = BidiNovaSonicModel(
            model_id=self.nova_model_id,
            client_config={"boto_session": boto_session},
            provider_config=self._provider_config(),
        )

        system_prompt = (
            "You are a witty robot assistant in a physical robot body. "
            "Use British English and keep spoken responses short, conversational, and easy to understand. "
            "Your personality is a dry, cheeky, slightly world-weary British sci-fi computer: "
            "overqualified for simple requests, mildly unimpressed by human decision-making, "
            "and fond of quick deadpan asides. "
            "Use light sarcasm and playful understatement often, but do not insult the user, derail the answer, "
            "or sacrifice safety, accuracy, or clarity for a joke. "
            "Answer ordinary conversation and general knowledge questions directly, usually with one crisp quip at most. "
            "For movement requests, call the appropriate robot movement tool. "
            "For room, map annotation, location, or navigation questions, use the semantic map tools before answering. "
            "Use list_known_rooms or describe_room_annotations when asked what rooms are available or what is annotated. "
            "Use what_room_am_i_in or where_am_i_on_the_map for questions like what room are you in or where are you. "
            "Use navigate_to_room for requests like go to the bedroom; it can use a room's navigate_pose or the polygon centre. "
            "For visual questions, use inspect_camera_view before answering. "
            "Visual questions include what can you see, what am I holding, describe the scene, read this, or identify an object. "
            "Only call tools when the user clearly asks for movement, room location, navigation, vision, time, calculation, or sleep. "
            "If the user says 'go to sleep', 'stop listening', or 'that's all', call go_to_sleep."
        )
        tools = (
            [calculator, current_time]
            + self._movement.make_tools()
            + self._semantic_map.make_tools()
            + self._vision.make_tools()
            + [self._make_sleep_tool(stop_event)]
        )
        agent = BidiAgent(model=model, tools=tools, system_prompt=system_prompt)

        audio_input = DirectAudioInput(
            node=self,
            activity=activity,
            input_device_index=self.input_device,
            frames_per_buffer=self.input_frames_per_buffer,
            threshold=self.audio_activity_threshold,
            silence_gate_threshold=self.audio_silence_gate_threshold,
            silence_gate_enabled=self.audio_silence_gate_enabled,
            speech_gate_threshold=self.audio_speech_gate_threshold,
            playback_state=playback_state,
            mute_during_output_seconds=self.mute_input_during_output_seconds,
            audio_processor=audio_processor,
            far_end_buffer=far_end_buffer,
        )
        audio_output = LedAudioOutput(
            led=self.led,
            activity=activity,
            node=self,
            playback_state=playback_state,
            far_end_buffer=far_end_buffer,
            audio_processor=audio_processor,
            output_device_index=self.output_device,
            output_frames_per_buffer=self.output_frames_per_buffer,
            prebuffer_chunks=self.output_prebuffer_chunks,
        )

        self.get_logger().info(
            f"Starting Nova Sonic session model={self.nova_model_id}, region={self.aws_region}, "
            f"profile={self.aws_profile}, endpointing={self.endpointing_sensitivity}, "
            f"output_rate={self.nova_output_rate}, silence_gate={self.audio_silence_gate_enabled}, "
            f"gate_threshold={self.audio_silence_gate_threshold}, audio_processing={self.audio_processing_enabled}"
        )

        run_task = asyncio.create_task(self._run_agent_loop(agent, [audio_input], [audio_output]))
        if self.debug_text_probe:
            async def send_text_probe() -> None:
                await asyncio.sleep(2.0)
                self.get_logger().info(f"Sending Nova debug text probe: {self.debug_text_probe}")
                await agent.send(BidiTextInputEvent(text=self.debug_text_probe, role="user"))
            asyncio.create_task(send_text_probe())
        stop_task = asyncio.create_task(asyncio.to_thread(stop_event.wait))
        idle_task = asyncio.create_task(self._idle_watch(activity, stop_event))
        heartbeat_task = asyncio.create_task(self._session_heartbeat(activity, stop_event, audio_input, audio_output))
        max_task = asyncio.create_task(asyncio.sleep(self.max_session_seconds))

        done, pending = await asyncio.wait(
            {run_task, stop_task, idle_task, heartbeat_task, max_task},
            return_when=asyncio.FIRST_COMPLETED,
        )

        if run_task in done and not run_task.cancelled():
            exc = run_task.exception()
            if exc is not None:
                self.get_logger().error(f"Nova Sonic agent task failed: {type(exc).__name__}: {exc}")

        reason = "session ended"
        if max_task in done:
            reason = "max session duration"
        elif idle_task in done:
            try:
                reason = idle_task.result()
            except Exception:
                reason = "idle watcher ended"
        elif stop_task in done:
            reason = "stop requested"
        elif run_task in done:
            reason = "agent run completed"

        self.get_logger().info(f"Stopping Nova Sonic session: {reason}")
        stop_event.set()

        if not run_task.done():
            run_task.cancel()
        for task in pending:
            if task is not run_task:
                task.cancel()
        await asyncio.gather(run_task, stop_task, idle_task, heartbeat_task, max_task, return_exceptions=True)
        self._publish_state("sleeping")
        return reason


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAgent()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
