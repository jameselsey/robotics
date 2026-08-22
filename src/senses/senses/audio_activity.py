"""Pure helpers for deciding whether microphone input extends a voice session."""


def counts_as_activity(
    amp: float,
    activity_threshold: float,
    silence_gate_enabled: bool,
    silence_gate_threshold: float,
    speech_detected: bool,
) -> bool:
    """Return whether a microphone frame can extend the local session."""
    threshold = activity_threshold
    if silence_gate_enabled:
        threshold = max(threshold, silence_gate_threshold)
    return amp >= threshold or speech_detected
