"""Select a PortAudio device without opening hardware."""


def select_audio_device(devices, *, name, fallback_index=-1, direction="input"):
    if direction not in {"input", "output"}:
        raise ValueError(f"Unsupported audio direction: {direction}")
    capability = "maxInputChannels" if direction == "input" else "maxOutputChannels"
    if name:
        for index, info in enumerate(devices):
            if int(info.get(capability, 0)) > 0 and name.lower() in info.get("name", "").lower():
                return index
    return int(fallback_index) if int(fallback_index) >= 0 else None
