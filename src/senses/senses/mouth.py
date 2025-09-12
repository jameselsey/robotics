import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from piper import PiperVoice
import wave
import numpy as np
import sounddevice as sd
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from gpiozero import PWMLED   # <-- NEW

LED_PIN = 19                  # <-- pick a PWM-capable pin (GPIO18 recommended)
LED_PWM_HZ = 400              # <-- LED PWM frequency
AMP_GAIN = 3.0                # <-- tweak: larger = brighter for same audio
SMOOTH_ALPHA = 0.25           # <-- 0..1; higher = smoother, less flicker

class Mouth(Node):
    def __init__(self):
        super().__init__('mouth')
        self.get_logger().info('👄 Mouth node has been started.')
        self.subscription = self.create_subscription(String, 'speech_output', self.say_text, 10)

        # Init TTS engine
        package_dir = Path(get_package_share_directory('senses'))
        voice = os.path.join(package_dir, 'resource', 'en_GB-southern_english_female-low.onnx')
        config = Path(str(voice) + '.json')
        self.piper = PiperVoice.load(str(voice), config_path=str(config))

        # Init LED (0..1 brightness)
        self.led = PWMLED(LED_PIN, frequency=LED_PWM_HZ)
        self.led.value = 0.0

    def say_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().info("🤐 Received empty message. Skipping TTS.")
            return

        self.get_logger().info(f"🗣️ Speaking: {text}")
        wav_path = "second_voice.wav"
        with wave.open(wav_path, "wb") as wav_file:
            self.piper.synthesize_wav(text, wav_file)

        try:
            self.play_with_led(wav_path)
        finally:
            # Make sure LED is off after speaking
            self.led.value = 0.0

    # Play audio and drive LED brightness from audio amplitude
    def play_with_led(self, filename: str):
        with wave.open(filename, 'rb') as wf:
            sample_rate = wf.getframerate()
            n_channels = wf.getnchannels()
            n_frames = wf.getnframes()
            raw = wf.readframes(n_frames)

        # Convert to mono float32 [-1, 1]
        audio = np.frombuffer(raw, dtype=np.int16)
        if n_channels == 2:
            audio = audio.reshape(-1, 2).mean(axis=1)
        audio = (audio.astype(np.float32) / 32768.0)

        # Stream out with a callback; update LED per block
        idx = 0
        prev_brightness = 0.0
        blocksize = 256  # smaller = more responsive LED, higher CPU

        def callback(outdata, frames, time, status):
            nonlocal idx, prev_brightness
            if status:
                # Optional: log underruns/overruns
                pass

            end = idx + frames
            chunk = audio[idx:end]
            if len(chunk) < frames:
                # Last partial block
                out = np.zeros((frames,), dtype=np.float32)
                out[:len(chunk)] = chunk
                outdata[:, 0] = out  # 1 channel output
                # LED fade out
                self.led.value = 0.0
                raise sd.CallbackStop()

            outdata[:, 0] = chunk

            # Compute RMS amplitude for this block and map to 0..1
            amp = float(np.sqrt(np.maximum(1e-12, np.mean(chunk * chunk))))
            brightness = min(1.0, AMP_GAIN * amp)

            # Exponential smoothing to reduce flicker
            brightness = SMOOTH_ALPHA * prev_brightness + (1.0 - SMOOTH_ALPHA) * brightness
            prev_brightness = brightness

            # Drive LED
            self.led.value = brightness

            idx = end

        # Open 1-channel stream (we converted to mono)
        with sd.OutputStream(samplerate=sample_rate, channels=1, dtype='float32',
                             blocksize=blocksize, callback=callback):
            # Wait until callback finishes
            sd.sleep(int(1000 * len(audio) / sample_rate) + 100)

def main(args=None):
    rclpy.init(args=args)
    node = Mouth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
