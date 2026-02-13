import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import json
import socket
import select
import threading
import time
import io
import wave
from pathlib import Path

import numpy as np
import pyaudio
import soundfile as sf
import whisper
from playsound import playsound


# ----------------------------
# Wyoming protocol helpers
# Wyoming is: JSON line + optional data bytes + optional payload bytes
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
    # Read JSON line
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

    # Optional data bytes (rarely used in wake)
    if data_length > 0:
        _ = _recv_exact(sock, data_length)

    payload = b""
    if payload_length > 0:
        payload = _recv_exact(sock, payload_length)

    return event_type, data, payload


class Ears(Node):
    def __init__(self):
        super().__init__("ears")
        self.publisher_ = self.create_publisher(String, "speech_input", 10)
        self.get_logger().info("Ears node started (Wyoming openWakeWord).")

        # ----------------------------
        # ROS params
        # ----------------------------
        self.declare_parameter("oww_host", "127.0.0.1")
        self.declare_parameter("oww_port", 10400)
        self.declare_parameter("wake_model_name", "computer")  # must match server model name
        self.declare_parameter("chunk_size", 1280)             # samples per read/send
        self.declare_parameter("input_device_name", "Brio")    # search for device by name
        self.declare_parameter("input_device_index", -1)       # fallback if name not found (-1 = use default)

        self.declare_parameter("silence_threshold", 300)        # mean abs int16
        self.declare_parameter("silence_duration", 0.8)         # seconds
        self.declare_parameter("min_record_seconds", 0.5)       # ignore very short captures
        self.declare_parameter("post_wake_cooldown", 0.25)      # seconds
        self.declare_parameter("flush_after_wake_chunks", 3)    # discard buffered audio after wake

        self.oww_host = str(self.get_parameter("oww_host").value)
        self.oww_port = int(self.get_parameter("oww_port").value)
        self.wake_model_name = str(self.get_parameter("wake_model_name").value)

        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.input_device_name = str(self.get_parameter("input_device_name").value)
        self.input_device_index = int(self.get_parameter("input_device_index").value)

        self.silence_threshold = int(self.get_parameter("silence_threshold").value)
        self.silence_duration = float(self.get_parameter("silence_duration").value)
        self.min_record_seconds = float(self.get_parameter("min_record_seconds").value)
        self.post_wake_cooldown = float(self.get_parameter("post_wake_cooldown").value)
        self.flush_after_wake_chunks = int(self.get_parameter("flush_after_wake_chunks").value)

        # Audio format (openWakeWord expects 16kHz mono int16)
        self.RATE = 16000
        self.CHANNELS = 1
        self.WIDTH_BYTES = 2  # int16
        self.FORMAT = pyaudio.paInt16

        # Resource paths
        package_dir = Path(get_package_share_directory("senses"))
        self.sound_path = str(package_dir / "resource" / "r2-sound-acknowledged.mp3")

        # Initialize audio + whisper
        self.pa = pyaudio.PyAudio()
        self.whisper_model = whisper.load_model("tiny")

        # Find audio input device by name
        device_index = self._find_audio_device(self.input_device_name, self.input_device_index)
        
        # Open a single mic stream (we will reuse this for both wake streaming + recording)
        self.mic_stream = self.pa.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.chunk_size,
            input_device_index=device_index,
        )

        # Background thread for wake loop
        self._stop = False
        self._thread = threading.Thread(target=self._wake_loop, daemon=True)
        self._thread.start()

    def _find_audio_device(self, device_name: str, fallback_index: int) -> int | None:
        """
        Find audio input device by name (case-insensitive substring match).
        Returns device index if found, fallback_index if not found, or None for default device.
        """
        self.get_logger().info(f"Searching for audio device containing '{device_name}'...")
        
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            name = info.get('name', '')
            max_input_channels = info.get('maxInputChannels', 0)
            
            # Check if this device has input capability and name matches
            if max_input_channels > 0 and device_name.lower() in name.lower():
                self.get_logger().info(f"✅ Found audio device: {name} (index={i})")
                return i
        
        # Device not found by name
        if fallback_index >= 0:
            self.get_logger().warn(
                f"⚠️  Device '{device_name}' not found, using fallback index {fallback_index}"
            )
            return fallback_index
        else:
            self.get_logger().warn(
                f"⚠️  Device '{device_name}' not found, using system default device"
            )
            return None

    def destroy_node(self):
        self._stop = True
        try:
            if getattr(self, "mic_stream", None):
                self.mic_stream.stop_stream()
                self.mic_stream.close()
        except Exception:
            pass
        try:
            if getattr(self, "pa", None):
                self.pa.terminate()
        except Exception:
            pass
        super().destroy_node()

    # ----------------------------
    # Wake loop: stream mic audio to Wyoming openWakeWord until detection
    # ----------------------------
    def _wake_loop(self):
        self.get_logger().info(f"Connecting to Wyoming openWakeWord at {self.oww_host}:{self.oww_port} ...")

        while rclpy.ok() and not self._stop:
            sock = None
            try:
                sock = socket.create_connection((self.oww_host, self.oww_port), timeout=5)
                sock.settimeout(None)

                # Request detection for a specific wake model name
                wyoming_send_event(sock, "detect", {"names": [self.wake_model_name]})

                # Start audio stream
                wyoming_send_event(
                    sock,
                    "audio-start",
                    {"rate": self.RATE, "width": self.WIDTH_BYTES, "channels": self.CHANNELS},
                )

                self.get_logger().info(f"👂 Listening for wake word '{self.wake_model_name}' ...")

                detected = False
                detected_name = None

                while rclpy.ok() and not self._stop:
                    pcm_bytes = self.mic_stream.read(self.chunk_size, exception_on_overflow=False)

                    wyoming_send_event(
                        sock,
                        "audio-chunk",
                        {"rate": self.RATE, "width": self.WIDTH_BYTES, "channels": self.CHANNELS},
                        payload=pcm_bytes,
                    )

                    # Non-blocking check for server response
                    r, _, _ = select.select([sock], [], [], 0)
                    if not r:
                        continue

                    event_type, data, _payload = wyoming_recv_event(sock)
                    if event_type == "detection":
                        detected = True
                        detected_name = data.get("name") or self.wake_model_name
                        break
                    elif event_type == "not-detected":
                        detected = False
                        break

                # Stop audio stream (good hygiene)
                try:
                    wyoming_send_event(sock, "audio-stop", {})
                except Exception:
                    pass

                if not detected:
                    continue

                self.get_logger().info(f"✅ Wake word detected! (model={detected_name})")
                try:
                    playsound(self.sound_path, block=False)
                except Exception:
                    pass

                # Flush a bit of buffered audio so we don't capture pre-wake audio
                for _ in range(max(0, self.flush_after_wake_chunks)):
                    _ = self.mic_stream.read(self.chunk_size, exception_on_overflow=False)

                # Record + transcribe command locally (REUSE mic_stream; do NOT open a 2nd capture device)
                wav_buffer = self._record_command_until_silence_from_existing_stream(
                    stream=self.mic_stream,
                    sample_rate=self.RATE,
                    chunk_size=int(self.RATE * 0.25),  # 250ms chunks
                    silence_threshold=self.silence_threshold,
                    silence_duration=self.silence_duration,
                )

                duration = self._wav_duration_seconds(wav_buffer)
                if duration < self.min_record_seconds:
                    self.get_logger().info("⏱️ No speech detected.")
                    time.sleep(self.post_wake_cooldown)
                    continue

                try:
                    transcript = self._transcribe_audio(wav_buffer)
                    if transcript:
                        self.get_logger().info(f"🗣️ You said: {transcript}")
                        msg = String()
                        msg.data = transcript
                        self.publisher_.publish(msg)
                    else:
                        self.get_logger().info("🤐 No transcribable speech detected.")
                except Exception as e:
                    self.get_logger().error(f"❌ Error during transcription: {e}")

                time.sleep(self.post_wake_cooldown)

            except Exception as e:
                self.get_logger().error(f"Wake loop error: {e}")
                time.sleep(1.0)
            finally:
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass

    # ----------------------------
    # Record until silence (using existing open stream)
    # ----------------------------
    def _record_command_until_silence_from_existing_stream(
        self,
        stream,
        sample_rate: int,
        chunk_size: int,
        silence_threshold: int,
        silence_duration: float,
    ) -> io.BytesIO:
        self.get_logger().info("🎙️ Listening for command (auto-stop on silence)...")

        frames: list[bytes] = []

        silence_limit_chunks = max(1, int(silence_duration / (chunk_size / sample_rate)))
        silent_chunks = 0

        while rclpy.ok() and not self._stop:
            data = stream.read(chunk_size, exception_on_overflow=False)
            frames.append(data)

            audio_np = np.frombuffer(data, dtype=np.int16)
            volume = float(np.abs(audio_np).mean())

            if volume < silence_threshold:
                silent_chunks += 1
            else:
                silent_chunks = 0

            if silent_chunks >= silence_limit_chunks:
                self.get_logger().info("🔇 Silence detected, stopping recording.")
                break

        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # int16
            wf.setframerate(sample_rate)
            wf.writeframes(b"".join(frames))

        wav_buffer.seek(0)
        return wav_buffer

    def _wav_duration_seconds(self, wav_buffer: io.BytesIO) -> float:
        wav_buffer.seek(0)
        with wave.open(wav_buffer, "rb") as wf:
            frames = wf.getnframes()
            rate = wf.getframerate()
        wav_buffer.seek(0)
        return frames / float(rate) if rate else 0.0

    # ----------------------------
    # Whisper transcription
    # ----------------------------
    def _transcribe_audio(self, wav_buffer: io.BytesIO) -> str:
        wav_buffer.seek(0)
        audio_array, sample_rate = sf.read(wav_buffer, dtype="float32")

        if len(audio_array.shape) > 1:
            audio_array = np.mean(audio_array, axis=1)

        if sample_rate != 16000:
            import librosa
            audio_array = librosa.resample(audio_array, orig_sr=sample_rate, target_sr=16000)

        audio_array = whisper.pad_or_trim(audio_array)
        mel = whisper.log_mel_spectrogram(audio_array).to(self.whisper_model.device)
        options = whisper.DecodingOptions(language="en", fp16=False)
        result = whisper.decode(self.whisper_model, mel, options)
        return (result.text or "").strip()


def main(args=None):
    rclpy.init(args=args)
    node = Ears()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
