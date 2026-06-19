"""LED and short sound-effect feedback for the voice agent."""

import subprocess
import threading
import time

import numpy as np
from playsound import playsound
from rclpy.node import Node

try:
    from gpiozero import PWMLED
except Exception:  # pragma: no cover - allows development away from robot GPIO
    PWMLED = None


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


def play_sound_with_led(node: Node, led: LedController, sound_path: str, label: str) -> None:
    """Play an MP3 and drive the LED from the decoded audio amplitude."""
    try:
        playsound(sound_path, block=False)
    except Exception as exc:
        node.get_logger().warn(f"Could not play {label} sound: {exc}")
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
                    led.audio_bytes(chunk)
                    time.sleep(len(chunk) / bytes_per_second)
        except Exception as exc:
            node.get_logger().debug(f"Could not drive LED from {label} sound: {exc}")
        finally:
            led.off()

    threading.Thread(target=follow_audio, daemon=True).start()
