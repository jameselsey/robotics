import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os
import struct
import io
import wave
import numpy as np
import soundfile as sf
import pyaudio
import pvporcupine
import whisper
import time

class Ears(Node):
    def __init__(self):
        super().__init__('ears')
        self.publisher_ = self.create_publisher(String, 'speech_input', 10)
        self.get_logger().info('Ears node has been started.')

        # Initialize Porcupine wake word engine
        package_dir = get_package_share_directory('senses')
        keyword_path = os.path.join(package_dir, 'resource', 'Hey-R-Two_en_raspberry-pi_v3_0_0.ppn')
        porcupine_access_key = os.environ.get("PORCUPINE_ACCESS_KEY")
        if not porcupine_access_key:
            self.get_logger().error("PORCUPINE_ACCESS_KEY environment variable not set.")
            return

        self.porcupine = pvporcupine.create(
            access_key=porcupine_access_key,
            keyword_paths=[keyword_path],
            sensitivities=[0.7]
        )

        # Initialize PyAudio
        self.pa = pyaudio.PyAudio()

        # Load Whisper model
        self.whisper_model = whisper.load_model("base")

        # Start listening
        self.listen_for_wake_word()

    def listen_for_wake_word(self):
        self.get_logger().info("👂 Listening for wake word...")
        try:
            while rclpy.ok():
                stream = self.pa.open(
                    rate=self.porcupine.sample_rate,
                    channels=1,
                    format=pyaudio.paInt16,
                    input=True,
                    frames_per_buffer=self.porcupine.frame_length,
                    input_device_index=0
                )

                try:
                    while True:
                        pcm = stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                        pcm_unpacked = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

                        if self.porcupine.process(pcm_unpacked) >= 0:
                            self.get_logger().info("✅ Wake word detected!")
                            break  # Exit inner loop to start recording

                finally:
                    stream.stop_stream()
                    stream.close()

                raw_audio = self.record_command(
                    self.porcupine.sample_rate,
                    int(self.porcupine.sample_rate * 0.5)
                )

                raw_audio.seek(0, io.SEEK_END)
                size_in_bytes = raw_audio.tell()
                raw_audio.seek(0)

                num_samples = size_in_bytes // 2
                if num_samples < self.porcupine.sample_rate * 2:
                    self.get_logger().info("⏱️ No speech detected.")
                    continue

                try:
                    transcript = self.transcribe_audio(raw_audio, self.porcupine.sample_rate)
                    if transcript:
                        self.get_logger().info(f"🗣️ You said: {transcript}")
                        msg = String()
                        msg.data = transcript
                        self.publisher_.publish(msg)
                    else:
                        self.get_logger().info("🤐 No transcribable speech detected.")
                except Exception as e:
                    self.get_logger().error(f"❌ Error during transcription: {e}")

        except KeyboardInterrupt:
            self.get_logger().info("👋 Exiting...")

        finally:
            self.pa.terminate()
            self.porcupine.delete()

    def record_command(self, sample_rate, chunk_size, silence_threshold=300, silence_duration=1.5):
        stream = self.pa.open(
            rate=sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=chunk_size,
            input_device_index=0
        )

        self.get_logger().info("🎙️ Listening for command (auto-stop on silence)...")
        frames = []
        silence_chunks = int(silence_duration / (chunk_size / sample_rate))
        silent_chunks = 0

        try:
            while True:
                data = stream.read(chunk_size, exception_on_overflow=False)
                frames.append(data)

                # Convert bytes to numpy int16 to compute volume
                audio_np = np.frombuffer(data, dtype=np.int16)
                volume = np.abs(audio_np).mean()

                if volume < silence_threshold:
                    silent_chunks += 1
                else:
                    silent_chunks = 0

                if silent_chunks > silence_chunks:
                    self.get_logger().info("🔇 Silence detected, stopping recording.")
                    break
        finally:
            stream.stop_stream()
            stream.close()

        # Wrap into WAV buffer
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16-bit
            wf.setframerate(sample_rate)
            wf.writeframes(b''.join(frames))
        wav_buffer.seek(0)

        return wav_buffer

    def transcribe_audio(self, wav_buffer, original_sample_rate):
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
        return result.text.strip()

def main(args=None):
    rclpy.init(args=args)
    node = Ears()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
