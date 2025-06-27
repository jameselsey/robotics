import pvporcupine
import pyaudio
import struct
import os
import whisper
import numpy as np
import soundfile as sf
import io
import time
import requests
import wave

# Load Whisper
whisper_model = whisper.load_model("base")

# Setup audio input
CHUNK_DURATION = 0.5  # seconds per chunk
LISTEN_TIMEOUT = 5    # seconds max to wait after wake word

def record_command(pa, sample_rate, chunk_size, silence_threshold=200, silence_duration=2.0):
    stream = pa.open(
        rate=sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=chunk_size
    )

    print("🎙️ Listening for command (auto-stop on silence)...")
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
                print("🔇 Silence detected, stopping recording.")
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

def transcribe_audio(wav_buffer, original_sample_rate):
    audio_array, sample_rate = sf.read(wav_buffer, dtype="float32")

    if len(audio_array.shape) > 1:
        audio_array = np.mean(audio_array, axis=1)

    if sample_rate != 16000:
        import librosa
        audio_array = librosa.resample(audio_array, orig_sr=sample_rate, target_sr=16000)

    audio_array = whisper.pad_or_trim(audio_array)
    mel = whisper.log_mel_spectrogram(audio_array).to(whisper_model.device)
    options = whisper.DecodingOptions(language="en", fp16=False)
    result = whisper.decode(whisper_model, mel, options)
    return result.text.strip()


def send_to_ollama(prompt):
    print(f"🤖 Sending to Ollama: {prompt}")
    response = requests.post(
        os.environ["OLLAMA_API_URL"],
        json={"prompt": prompt}
    )
    print(f"🧠 Ollama replied: {response.text}")

def main():
    porcupine = pvporcupine.create(
        access_key=os.environ["PORCUPINE_ACCESS_KEY"],
        keyword_paths=["Hey-R-two_en_mac_v3_0_0.ppn"],
        sensitivities=[0.7]
    )

    pa = pyaudio.PyAudio()
    stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=porcupine.frame_length
    )

    print("👂 Listening for wake word...")

    try:
        while True:
            pcm = stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm_unpacked = struct.unpack_from("h" * porcupine.frame_length, pcm)

            if porcupine.process(pcm_unpacked) >= 0:
                print("✅ Wake word detected!")
                raw_audio = record_command(pa, porcupine.sample_rate, int(porcupine.sample_rate * CHUNK_DURATION))
                raw_audio.seek(0, io.SEEK_END)
                size_in_bytes = raw_audio.tell()
                raw_audio.seek(0)

                # 2 bytes per sample, mono channel
                num_samples = size_in_bytes // 2
                if num_samples < porcupine.sample_rate * 2:
                    print("⏱️ No speech detected.")
                    continue

                try:
                    transcript = transcribe_audio(raw_audio, porcupine.sample_rate)
                    if transcript:
                        print(f"🗣️ You said: {transcript}")
                        send_to_ollama(transcript)
                    else:
                        print("🤐 No transcribable speech detected.")
                except Exception as e:
                    print(f"❌ Error during transcription: {e}")

    except KeyboardInterrupt:
        print("👋 Exiting...")
    finally:
        stream.stop_stream()
        stream.close()
        pa.terminate()
        porcupine.delete()

if __name__ == "__main__":
    main()
