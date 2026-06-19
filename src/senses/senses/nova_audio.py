"""Nova Sonic audio input/output helpers, including WebRTC echo cancellation."""

import asyncio
import base64
import queue
import threading
import time

import numpy as np
import pyaudio
from rclpy.node import Node
from strands.experimental.bidi.types.events import (
    BidiAudioInputEvent,
    BidiAudioStreamEvent,
    BidiInterruptionEvent,
    BidiResponseStartEvent,
)

from senses.audio_feedback import LedController

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
        self._response_chunk_count = 0
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
            self._response_chunk_count = 0
        elif isinstance(event, BidiAudioStreamEvent):
            data = base64.b64decode(event["audio"])
            self.output_chunk_count += 1
            self.last_output_bytes = len(data)
            self._response_chunk_count += 1
            self._buffer.put(data)
            if not self._playback_started and self._response_chunk_count >= self._prebuffer_chunks:
                self._playback_started = True
                self._node.get_logger().info(
                    f"Nova speaker prebuffer ready after {self._response_chunk_count} chunks."
                )
            self._activity.mark()
            now = time.monotonic()
            if now - self._last_log >= 2.0:
                self._last_log = now
                self._node.get_logger().debug(
                    f"Nova output chunks={self.output_chunk_count}, "
                    f"last_bytes={self.last_output_bytes}"
                )
        elif isinstance(event, BidiInterruptionEvent):
            self._node.get_logger().info(
                "Nova interruption event ignored because local barge-in is disabled."
            )

    def _callback(self, _in_data, frame_count: int, *_args):
        self.playback_callback_count += 1
        byte_count = frame_count * self._channels * pyaudio.get_sample_size(pyaudio.paInt16)
        if not self._playback_started:
            data = b"\x00" * byte_count
        else:
            data = self._buffer.get(byte_count)
        # Feed exactly what we hand to the speaker into the far-end reference for echo cancellation.
        if data:
            samples = np.frombuffer(data, dtype=np.int16)
            if self._far_end_buffer is not None:
                self._far_end_buffer.put(samples)
            if samples.size and np.any(samples):
                self._playback_state.mark_output_audio()
        self._led.audio_bytes(data)
        return (data, pyaudio.paContinue)
