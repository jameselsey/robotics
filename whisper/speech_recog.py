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
import signal
import sys
import pyttsx3


# Load Whisper
whisper_model = whisper.load_model("base")

# load TTS engine
tts_engine = pyttsx3.init()

# Setup constants
CHUNK_DURATION = 0.5  # seconds per chunk
SILENCE_THRESHOLD = 200
SILENCE_DURATION = 2.0

def handle_interrupt(sig, frame):
    print("\n👋 Caught interrupt, exiting cleanly...")
    sys.exit(0)

signal.signal(signal.SIGINT, handle_interrupt)

def speak(text: str):
    print(f"🗣️ Speaking: {text}")
    tts_engine.say(text)
    tts_engine.runAndWait()

def record_command(pa, sample_rate, chunk_size, silence_threshold=SILENCE_THRESHOLD, silence_duration=SILENCE_DURATION):
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
        os.environ["OLLAMA_API_URL"].rstrip('/') + "/api/generate",
        json={
            "model": "tinyllama",
            "prompt": prompt,
            "stream": False
        }
    )
    if response.ok:
        response_json = response.json()
        print(f"🧠 Ollama replied: {response_json['response']}")
        speak(response_json['response'])
    else:
        print(f"❌ Ollama error: {response.status_code} {response.text}")

def main():
    porcupine = pvporcupine.create(
        access_key=os.environ["PORCUPINE_ACCESS_KEY"],
        keyword_paths=["Hey-R-two_en_mac_v3_0_0.ppn"],
        sensitivities=[0.7]
    )

    pa = pyaudio.PyAudio()

    print("👂 Listening for wake word...")

    try:
        while True:
            stream = pa.open(
                rate=porcupine.sample_rate,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                frames_per_buffer=porcupine.frame_length
            )

            try:
                while True:
                    pcm = stream.read(porcupine.frame_length, exception_on_overflow=False)
                    pcm_unpacked = struct.unpack_from("h" * porcupine.frame_length, pcm)

                    if porcupine.process(pcm_unpacked) >= 0:
                        print("✅ Wake word detected!")
                        break  # Exit inner loop to start recording

            finally:
                stream.stop_stream()
                stream.close()

            raw_audio = record_command(
                pa,
                porcupine.sample_rate,
                int(porcupine.sample_rate * CHUNK_DURATION)
            )

            raw_audio.seek(0, io.SEEK_END)
            size_in_bytes = raw_audio.tell()
            raw_audio.seek(0)

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
        pa.terminate()
        porcupine.delete()

if __name__ == "__main__":
    main()
