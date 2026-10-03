"""ROS2 voice agent node backed by Strands and Amazon Nova Sonic."""

import asyncio
import json
import logging
import select
import socket
import threading
import time
from pathlib import Path

import pyaudio
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from strands import tool
from strands.experimental.bidi.types.events import BidiTextInputEvent
from strands_tools import calculator, current_time

from senses.audio_devices import select_audio_device
from senses.audio_feedback import LedController, play_sound_with_led
from senses.conversation_session import run_agent_io, run_session
from senses.movement_tools import MovementController
from senses.nova_audio import (
    ActivityTracker,
    DirectAudioInput,
    FarEndReferenceBuffer,
    LedAudioOutput,
    PlaybackState,
)
from senses.nova_backend import create_nova_agent
from senses.semantic_map_tools import SemanticMapController
from senses.vision_tools import VisionController
from senses.voice_config import declare_voice_config
from senses.wyoming_protocol import wyoming_recv_event, wyoming_send_event

try:
    from pywebrtc_audio import AudioProcessor
except Exception:  # pragma: no cover - lets the node run without AEC installed
    AudioProcessor = None


class VoiceAgent(Node):
    def __init__(self):
        super().__init__("voice_agent")
        self.get_logger().info("Voice agent node started (wake word + Nova Sonic bidi).")

        self.state_pub = self.create_publisher(String, "voice_state", 10)
        self.transcript_pub = self.create_publisher(String, "voice_transcript", 10)
        self.create_subscription(String, "voice_control", self._voice_control_callback, 10)
        self._movement = MovementController(self)
        self._manual_wake_event = threading.Event()
        self._conversation_stop_event: threading.Event | None = None
        self._conversation_lock = threading.Lock()

        self.config = declare_voice_config(self)
        self.RATE = 16000
        self.CHANNELS = 1
        self.WIDTH_BYTES = 2
        self.FORMAT = pyaudio.paInt16

        package_dir = Path(get_package_share_directory("senses"))
        self.sound_path = str(package_dir / "resource" / "r2-sound-acknowledged.mp3")
        self.stop_sound_path = str(package_dir / "resource" / "stop-listening.mp3")
        if not self.config.rooms_config_path:
            self.config.rooms_config_path = str(package_dir / "config" / "rooms.yaml")
        self._semantic_map = SemanticMapController(
            self,
            self.config.rooms_config_path,
            navigation_log_path=self.config.navigation_log_path,
        )
        self._vision = VisionController(
            self,
            topic=self.config.vision_topic,
            model_id=self.config.vision_model_id,
            aws_profile=self.config.aws_profile,
            aws_region=self.config.aws_region,
            enabled=self.config.vision_enabled,
            frame_timeout_seconds=self.config.vision_frame_timeout_seconds,
        )

        try:
            logging.getLogger("strands").setLevel(
                getattr(logging, self.config.strands_log_level, logging.DEBUG)
            )
            logging.getLogger("aws_sdk_bedrock_runtime").setLevel(logging.DEBUG)
            logging.getLogger("smithy").setLevel(logging.DEBUG)
        except Exception:
            pass

        self.pa = pyaudio.PyAudio()
        self._log_audio_devices()
        self.input_device = self._find_audio_device(
            self.config.input_device_name, self.config.input_device_index
        )
        self.output_device = self._find_audio_device(
            self.config.output_device_name, self.config.output_device_index, direction="output"
        )
        self.led = LedController(
            self,
            pin=self.config.led_pin,
            pwm_hz=self.config.led_pwm_hz,
            amp_gain=self.config.led_amp_gain,
            smooth_alpha=self.config.led_smooth_alpha,
        )

        self._stop = False
        self._thread = threading.Thread(target=self._wake_loop, daemon=True)
        self._thread.start()

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Voice state: {state}")

    def _voice_control_callback(self, msg: String) -> None:
        command = (msg.data or "").strip().lower()
        if command == "wake":
            self.get_logger().info("Voice control wake requested")
            self._manual_wake_event.set()
            return
        if command == "stop":
            self.get_logger().info("Voice control stop requested")
            with self._conversation_lock:
                stop_event = self._conversation_stop_event
            if stop_event is not None:
                stop_event.set()
            else:
                self._manual_wake_event.clear()
            return
        self.get_logger().warn(f"Ignoring unknown voice control command: {msg.data!r}")

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

    def _find_audio_device(self, device_name, fallback_index, direction="input"):
        devices = [self.pa.get_device_info_by_index(i) for i in range(self.pa.get_device_count())]
        index = select_audio_device(
            devices, name=device_name, fallback_index=fallback_index, direction=direction
        )
        if index is None:
            self.get_logger().warn(f"Using system default {direction} device")
        else:
            self.get_logger().info(f"Selected audio {direction} device index={index}")
        return index

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
                sock = socket.create_connection(
                    (self.config.oww_host, self.config.oww_port), timeout=5
                )
                sock.settimeout(None)
                wyoming_send_event(sock, "detect", {"names": [self.config.wake_model_name]})
                wyoming_send_event(
                    sock,
                    "audio-start",
                    {"rate": self.RATE, "width": self.WIDTH_BYTES, "channels": self.CHANNELS},
                )

                mic_stream = self.pa.open(
                    format=self.FORMAT,
                    channels=self.CHANNELS,
                    rate=self.RATE,
                    input=True,
                    frames_per_buffer=self.config.chunk_size,
                    input_device_index=self.input_device,
                )

                self.get_logger().info(
                    f"Listening for wake word '{self.config.wake_model_name}'..."
                )
                detected = False
                detected_name = None

                while rclpy.ok() and not self._stop:
                    if self._manual_wake_event.is_set():
                        self._manual_wake_event.clear()
                        detected = True
                        detected_name = "controller"
                        break

                    pcm_bytes = mic_stream.read(self.config.chunk_size, exception_on_overflow=False)
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
                        detected_name = data.get("name") or self.config.wake_model_name
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
                play_sound_with_led(self, self.led, self.sound_path, "wake")
                time.sleep(self.config.wake_ack_delay)

                conversation_reason = asyncio.run(self._run_conversation())
                if conversation_reason == "stop requested":
                    play_sound_with_led(self, self.led, self.stop_sound_path, "stop")
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

    def _make_sleep_tool(self, stop_event: threading.Event):
        @tool
        def go_to_sleep() -> str:
            """End this voice conversation and return the robot to wake-word listening mode."""
            stop_event.set()
            return "Going back to sleep."

        return go_to_sleep

    async def _session_heartbeat(
        self,
        activity: ActivityTracker,
        stop_event: threading.Event,
        audio_input: DirectAudioInput,
        audio_output: LedAudioOutput,
    ) -> str:
        while not stop_event.is_set():
            await asyncio.sleep(max(1.0, self.config.session_heartbeat_seconds))
            self.get_logger().debug(
                f"Nova heartbeat idle={activity.idle_seconds():.1f}s, "
                f"input_chunks={audio_input.chunk_count}, active_input={audio_input.active_chunk_count}, "
                f"gated_input={audio_input.gated_chunk_count}, output_muted={audio_input.output_muted_chunk_count}, "
                f"aec={audio_input.aec_processed_chunk_count}, speech_prob={audio_input.speech_probability:.2f}, "
                f"last_amp={audio_input.last_amp:.1f}, output_chunks={audio_output.output_chunk_count}, "
                f"playback_callbacks={audio_output.playback_callback_count}"
            )
        return "heartbeat stopped"

    def _publish_transcript(self, transcript):
        message = String()
        message.data = json.dumps(transcript, sort_keys=True)
        self.transcript_pub.publish(message)

    async def _run_conversation(self):
        self._publish_state("conversation")
        stop_event = threading.Event()
        with self._conversation_lock:
            self._conversation_stop_event = stop_event
        try:
            return await self._run_nova_session(stop_event)
        finally:
            stop_event.set()
            with self._conversation_lock:
                if self._conversation_stop_event is stop_event:
                    self._conversation_stop_event = None
            self._publish_state("sleeping")

    async def _run_nova_session(self, stop_event):
        activity = ActivityTracker()
        playback_state = PlaybackState()
        far_end_buffer = FarEndReferenceBuffer()
        audio_processor = None
        if self.config.audio_processing_enabled:
            if AudioProcessor is None:
                self.get_logger().warn(
                    "pywebrtc-audio is not installed; AEC/noise suppression disabled."
                )
            else:
                # pywebrtc-audio wraps WebRTC AEC/NS/AGC. It expects 10 ms mono PCM frames at 16 kHz.
                audio_processor = AudioProcessor(
                    sample_rate=16000,
                    num_channels=1,
                    echo_cancellation=True,
                    noise_suppression=True,
                    auto_gain_control=True,
                    stream_delay_ms=self.config.audio_processing_stream_delay_ms,
                )
                self.get_logger().info(
                    f"WebRTC audio processing enabled: AEC+NS+AGC, "
                    f"stream_delay_ms={self.config.audio_processing_stream_delay_ms}"
                )

        tools = (
            [calculator, current_time]
            + self._movement.make_tools()
            + self._semantic_map.make_tools()
            + self._vision.make_tools()
            + [self._make_sleep_tool(stop_event)]
        )
        agent = create_nova_agent(self.config, tools)

        audio_input = DirectAudioInput(
            node=self,
            activity=activity,
            input_device_index=self.input_device,
            frames_per_buffer=self.config.input_frames_per_buffer,
            threshold=self.config.audio_activity_threshold,
            silence_gate_threshold=self.config.audio_silence_gate_threshold,
            silence_gate_enabled=self.config.audio_silence_gate_enabled,
            speech_gate_threshold=self.config.audio_speech_gate_threshold,
            playback_state=playback_state,
            mute_during_output_seconds=self.config.mute_input_during_output_seconds,
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
            output_frames_per_buffer=self.config.output_frames_per_buffer,
            prebuffer_chunks=self.config.output_prebuffer_chunks,
        )

        self.get_logger().info(
            f"Starting Nova Sonic session model={self.config.nova_model_id}, region={self.config.aws_region}, "
            f"profile={self.config.aws_profile}, endpointing={self.config.endpointing_sensitivity}, "
            f"output_rate={self.config.nova_output_rate}, silence_gate={self.config.audio_silence_gate_enabled}, "
            f"gate_threshold={self.config.audio_silence_gate_threshold}, audio_processing={self.config.audio_processing_enabled}"
        )

        probe = None
        if self.config.debug_text_probe:

            async def send_text_probe():
                await asyncio.sleep(2.0)
                self.get_logger().info(
                    f"Sending Nova debug text probe: {self.config.debug_text_probe}"
                )
                await agent.send(BidiTextInputEvent(text=self.config.debug_text_probe, role="user"))

            probe = send_text_probe()
        return await run_session(
            run_agent_io(
                agent,
                [audio_input],
                [audio_output],
                logger=self.get_logger(),
                publish_transcript=self._publish_transcript,
            ),
            stop_event=stop_event,
            activity=activity,
            idle_timeout_seconds=self.config.idle_timeout_seconds,
            max_session_seconds=self.config.max_session_seconds,
            heartbeat=self._session_heartbeat(activity, stop_event, audio_input, audio_output),
            probe=probe,
            logger=self.get_logger(),
        )


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
