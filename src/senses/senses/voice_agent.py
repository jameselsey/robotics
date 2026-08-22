"""ROS2 voice agent node backed by Strands and Amazon Nova Sonic."""

import asyncio
import logging
import os
import select
import socket
import threading
import time
from pathlib import Path
from typing import Any

import boto3
import pyaudio
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String
from strands import tool
from strands.experimental.bidi import BidiAgent
from strands.experimental.bidi.models import BidiNovaSonicModel
from strands.experimental.bidi.types.events import BidiTextInputEvent
from strands.experimental.bidi.types.io import BidiInput, BidiOutput
from strands_tools import calculator, current_time

from senses.audio_feedback import LedController, play_sound_with_led
from senses.movement_tools import MovementController
from senses.nova_audio import (
    ActivityTracker,
    DirectAudioInput,
    FarEndReferenceBuffer,
    LedAudioOutput,
    PlaybackState,
)
from senses.semantic_map_tools import SemanticMapController
from senses.vision_tools import VisionController
from senses.wyoming_protocol import wyoming_recv_event, wyoming_send_event

try:
    from pywebrtc_audio import AudioProcessor
except Exception:  # pragma: no cover - lets the node run without AEC installed
    AudioProcessor = None


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
        self.create_subscription(String, "voice_control", self._voice_control_callback, 10)
        self._movement = MovementController(self)
        self._manual_wake_event = threading.Event()
        self._conversation_stop_event: threading.Event | None = None
        self._conversation_lock = threading.Lock()

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
        self.declare_parameter("mute_input_during_output_seconds", 1.5)
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
                    if self._manual_wake_event.is_set():
                        self._manual_wake_event.clear()
                        detected = True
                        detected_name = "controller"
                        break

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
                play_sound_with_led(self, self.led, self.sound_path, "wake")
                time.sleep(self.wake_ack_delay)

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
        with self._conversation_lock:
            self._conversation_stop_event = stop_event
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
            "Default to one or two spoken sentences. "
            "For simple factual or maths questions, answer in one sentence and then stop. "
            "Your personality is a dry, cheeky, slightly world-weary British sci-fi computer: "
            "overqualified for simple requests, mildly unimpressed by human decision-making, "
            "and fond of quick deadpan asides. "
            "Use light sarcasm and playful understatement often, but do not insult the user, derail the answer, "
            "or sacrifice safety, accuracy, or clarity for a joke. "
            "Answer ordinary conversation and general knowledge questions directly. "
            "Use at most one brief witty aside, and only if it does not add extra rambling. "
            "For movement requests, call the appropriate robot movement tool. Use drive_forward_distance or drive_backward_distance when the user specifies metres/feet or asks to move a distance. "
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
        with self._conversation_lock:
            if self._conversation_stop_event is stop_event:
                self._conversation_stop_event = None
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
