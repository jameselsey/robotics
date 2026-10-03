"""Deployment configuration and audio selection without ROS or hardware."""

from types import SimpleNamespace

import pytest
from senses.audio_devices import select_audio_device
from senses.nova_backend import provider_config
from senses.voice_config import declare_voice_config, parameter_defaults


class Parameters:
    def __init__(self, **overrides):
        self.values = overrides

    def declare_parameter(self, name, default):
        self.values.setdefault(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


def test_environment_defaults_and_ros_overrides(monkeypatch):
    monkeypatch.setenv("AWS_REGION", "eu-west-2")
    monkeypatch.setenv("ROS_HOME", "/state")
    settings = declare_voice_config(
        Parameters(
            aws_region="us-east-1",
            vision_enabled="false",
            audio_processing_stream_delay_ms=-1,
            output_frames_per_buffer=320,
            endpointing_sensitivity="low",
            debug_text_probe=" test ",
        )
    )
    assert parameter_defaults()["aws_region"] == "eu-west-2"
    assert settings.aws_region == "us-east-1"
    assert settings.navigation_log_path == "/state/robopi/navigation_events.jsonl"
    assert settings.vision_enabled is False
    assert settings.audio_processing_stream_delay_ms == 20
    assert settings.debug_text_probe == "test"
    assert provider_config(settings)["turn_detection"] == {"endpointingSensitivity": "LOW"}
    settings.nova_model_id = "amazon.nova-sonic-v1:0"
    assert "turn_detection" not in provider_config(settings)


@pytest.mark.parametrize("direction,expected", [("input", 1), ("output", 0)])
def test_device_name_matches_only_correct_direction(direction, expected):
    devices = [
        {"name": "USB speaker", "maxInputChannels": 0, "maxOutputChannels": 2},
        {"name": "USB microphone", "maxInputChannels": 1, "maxOutputChannels": 0},
    ]
    assert select_audio_device(devices, name="usb", direction=direction) == expected
    assert select_audio_device(devices, name="missing", fallback_index=3) == 3
    assert select_audio_device(devices, name="missing") is None
    with pytest.raises(ValueError):
        select_audio_device(devices, name="usb", direction="other")
