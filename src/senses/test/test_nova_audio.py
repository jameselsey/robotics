"""Unit tests for Nova audio session activity detection."""

from senses.audio_activity import counts_as_activity


def test_gated_background_noise_does_not_extend_session():
    assert not counts_as_activity(400.0, 250.0, True, 650.0, False)


def test_audio_above_silence_gate_extends_session():
    assert counts_as_activity(700.0, 250.0, True, 650.0, False)


def test_webrtc_speech_extends_session_below_amplitude_gate():
    assert counts_as_activity(100.0, 250.0, True, 650.0, True)


def test_activity_threshold_applies_when_silence_gate_is_disabled():
    assert counts_as_activity(300.0, 250.0, False, 650.0, False)
