"""Voice parameter defaults and typed settings, independent of ROS and devices."""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def parse_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def parameter_defaults(environ=None) -> dict[str, Any]:
    """Read deployment environment defaults when a voice node is constructed."""
    environ = os.environ if environ is None else environ
    defaults = {
        "oww_host": "127.0.0.1",
        "oww_port": 10400,
        "wake_model_name": "computer",
        "chunk_size": 1280,
        "input_device_name": "Brio",
        "input_device_index": -1,
        "output_device_index": -1,
        "wake_ack_delay": 0.25,
        "aws_profile": environ.get("AWS_PROFILE", "default"),
        "aws_region": environ.get("AWS_REGION") or environ.get("AWS_DEFAULT_REGION", "us-east-1"),
        "nova_model_id": environ.get("NOVA_SONIC_MODEL_ID", "amazon.nova-2-sonic-v1:0"),
        "nova_voice": environ.get("NOVA_SONIC_VOICE", "amy"),
        "nova_output_rate": int(environ.get("NOVA_SONIC_OUTPUT_RATE", "16000")),
        "endpointing_sensitivity": environ.get("NOVA_SONIC_ENDPOINTING_SENSITIVITY", "LOW"),
        "temperature": float(environ.get("NOVA_SONIC_TEMPERATURE", "0.7")),
        "top_p": float(environ.get("NOVA_SONIC_TOP_P", "0.9")),
        "max_tokens": int(environ.get("NOVA_SONIC_MAX_TOKENS", "1024")),
        "idle_timeout_seconds": 45.0,
        "max_session_seconds": 420.0,
        "audio_activity_threshold": 250.0,
        "audio_silence_gate_enabled": True,
        "audio_silence_gate_threshold": 650.0,
        "audio_speech_gate_threshold": 0.6,
        "audio_processing_enabled": True,
        "audio_processing_stream_delay_ms": -1,
        "debug_text_probe": environ.get("NOVA_SONIC_DEBUG_TEXT_PROBE", ""),
        "session_heartbeat_seconds": 5.0,
        "strands_log_level": environ.get("STRANDS_LOG_LEVEL", "DEBUG"),
        "input_frames_per_buffer": 160,
        "output_frames_per_buffer": 160,
        "output_prebuffer_chunks": 0,
        "mute_input_during_output_seconds": 1.5,
        "rooms_config_path": "",
        "vision_enabled": True,
        "vision_topic": environ.get("VISION_TOPIC", "/image_viz/compressed"),
        "vision_model_id": environ.get("VISION_MODEL_ID", "amazon.nova-lite-v1:0"),
        "vision_frame_timeout_seconds": 3.0,
        "led_pin": 19,
        "led_pwm_hz": 400,
        "led_amp_gain": 3.0,
        "led_smooth_alpha": 0.25,
        "output_device_name": "",
        "navigation_log_path": str(
            Path(environ.get("ROS_HOME") or Path.home() / ".ros")
            / "robopi"
            / "navigation_events.jsonl"
        ),
    }
    return defaults


@dataclass
class VoiceConfig:
    """Settings shared by conversation, inference, audio, and ROS adapters."""

    oww_host: str
    oww_port: int
    wake_model_name: str
    chunk_size: int
    input_device_name: str
    input_device_index: int
    output_device_index: int
    wake_ack_delay: float
    aws_profile: str
    aws_region: str
    nova_model_id: str
    nova_voice: str
    nova_output_rate: int
    endpointing_sensitivity: str
    temperature: float
    top_p: float
    max_tokens: int
    idle_timeout_seconds: float
    max_session_seconds: float
    audio_activity_threshold: float
    audio_silence_gate_enabled: bool
    audio_silence_gate_threshold: float
    audio_speech_gate_threshold: float
    audio_processing_enabled: bool
    debug_text_probe: str
    session_heartbeat_seconds: float
    strands_log_level: str
    input_frames_per_buffer: int
    output_frames_per_buffer: int
    audio_processing_stream_delay_ms: int
    output_prebuffer_chunks: int
    mute_input_during_output_seconds: float
    rooms_config_path: str
    vision_enabled: bool
    vision_topic: str
    vision_model_id: str
    vision_frame_timeout_seconds: float
    output_device_name: str
    navigation_log_path: str
    led_pin: int
    led_pwm_hz: int
    led_amp_gain: float
    led_smooth_alpha: float


def declare_voice_config(node) -> VoiceConfig:
    """Declare the existing ROS parameters and normalize their selected values."""
    for name, default in parameter_defaults().items():
        node.declare_parameter(name, default)
    values = {}
    values["oww_host"] = str(node.get_parameter("oww_host").value)
    values["oww_port"] = int(node.get_parameter("oww_port").value)
    values["wake_model_name"] = str(node.get_parameter("wake_model_name").value)
    values["chunk_size"] = int(node.get_parameter("chunk_size").value)
    values["input_device_name"] = str(node.get_parameter("input_device_name").value)
    values["input_device_index"] = int(node.get_parameter("input_device_index").value)
    values["output_device_index"] = int(node.get_parameter("output_device_index").value)
    values["wake_ack_delay"] = float(node.get_parameter("wake_ack_delay").value)
    values["aws_profile"] = str(node.get_parameter("aws_profile").value)
    values["aws_region"] = str(node.get_parameter("aws_region").value)
    values["nova_model_id"] = str(node.get_parameter("nova_model_id").value)
    values["nova_voice"] = str(node.get_parameter("nova_voice").value)
    values["nova_output_rate"] = int(node.get_parameter("nova_output_rate").value)
    values["endpointing_sensitivity"] = str(
        node.get_parameter("endpointing_sensitivity").value
    ).upper()
    values["temperature"] = float(node.get_parameter("temperature").value)
    values["top_p"] = float(node.get_parameter("top_p").value)
    values["max_tokens"] = int(node.get_parameter("max_tokens").value)
    values["idle_timeout_seconds"] = float(node.get_parameter("idle_timeout_seconds").value)
    values["max_session_seconds"] = float(node.get_parameter("max_session_seconds").value)
    values["audio_activity_threshold"] = float(node.get_parameter("audio_activity_threshold").value)
    values["audio_silence_gate_enabled"] = parse_bool(
        node.get_parameter("audio_silence_gate_enabled").value
    )
    values["audio_silence_gate_threshold"] = float(
        node.get_parameter("audio_silence_gate_threshold").value
    )
    values["audio_speech_gate_threshold"] = float(
        node.get_parameter("audio_speech_gate_threshold").value
    )
    values["audio_processing_enabled"] = parse_bool(
        node.get_parameter("audio_processing_enabled").value
    )
    values["debug_text_probe"] = str(node.get_parameter("debug_text_probe").value).strip()
    values["session_heartbeat_seconds"] = float(
        node.get_parameter("session_heartbeat_seconds").value
    )
    values["strands_log_level"] = str(node.get_parameter("strands_log_level").value).upper()
    values["input_frames_per_buffer"] = int(node.get_parameter("input_frames_per_buffer").value)
    values["output_frames_per_buffer"] = int(node.get_parameter("output_frames_per_buffer").value)
    values["audio_processing_stream_delay_ms"] = int(
        node.get_parameter("audio_processing_stream_delay_ms").value
    )
    if values["audio_processing_stream_delay_ms"] < 0:
        values["audio_processing_stream_delay_ms"] = int(
            values["output_frames_per_buffer"] / 16000 * 1000
        )
    values["output_prebuffer_chunks"] = int(node.get_parameter("output_prebuffer_chunks").value)
    values["mute_input_during_output_seconds"] = float(
        node.get_parameter("mute_input_during_output_seconds").value
    )
    values["rooms_config_path"] = str(node.get_parameter("rooms_config_path").value).strip()
    values["vision_enabled"] = parse_bool(node.get_parameter("vision_enabled").value)
    values["vision_topic"] = str(node.get_parameter("vision_topic").value).strip()
    values["vision_model_id"] = str(node.get_parameter("vision_model_id").value).strip()
    values["vision_frame_timeout_seconds"] = float(
        node.get_parameter("vision_frame_timeout_seconds").value
    )
    values["output_device_name"] = str(node.get_parameter("output_device_name").value)
    values["navigation_log_path"] = str(node.get_parameter("navigation_log_path").value)
    values["led_pin"] = int(node.get_parameter("led_pin").value)
    values["led_pwm_hz"] = int(node.get_parameter("led_pwm_hz").value)
    values["led_amp_gain"] = float(node.get_parameter("led_amp_gain").value)
    values["led_smooth_alpha"] = float(node.get_parameter("led_smooth_alpha").value)
    return VoiceConfig(**values)
