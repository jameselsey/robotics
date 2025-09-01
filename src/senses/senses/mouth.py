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

class Mouth(Node):
    def __init__(self):
        super().__init__('mouth')
        self.get_logger().info('👄 Mouth node has been started.')

        # Subscribe to brain's output
        self.subscription = self.create_subscription(
            String,
            'speech_output',
            self.say_text,
            10
        )

        # Initialize TTS engine
        package_dir = Path(get_package_share_directory('senses'))
        voice = os.path.join(package_dir, 'resource', 'en_GB-southern_english_female-low.onnx')
        config = Path(str(voice) + '.json')
        self.piper = PiperVoice.load(str(voice), config_path=str(config))


    def say_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            self.get_logger().info("🤐 Received empty message. Skipping TTS.")
            return

        self.get_logger().info(f"🗣️ Speaking: {text}")
        second_voice_file = "second_voice.wav"
        with wave.open(second_voice_file, "wb") as wav_file:
            self.piper.synthesize_wav(text, wav_file)
            play(second_voice_file)


# Convenience function to play a filename
def play(target_filename):
    # Play the wav using sounddevice
    with wave.open(target_filename, 'rb') as wf:
        sample_rate = wf.getframerate()
        n_channels = wf.getnchannels()
        n_frames = wf.getnframes()
        audio = wf.readframes(n_frames)
        # Convert byte data to numpy array
        audio_np = np.frombuffer(audio, dtype=np.int16)
        # If stereo, reshape for sounddevice
        if n_channels == 2:
            audio_np = audio_np.reshape(-1, 2)
        # sounddevice expects float32 in [-1, 1]
        audio_np = audio_np.astype(np.float32) / 32768.0
        sd.play(audio_np, sample_rate)
        sd.wait()

def main(args=None):
    rclpy.init(args=args)
    node = Mouth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
