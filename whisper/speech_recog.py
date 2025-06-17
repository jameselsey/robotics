import pvporcupine
import pyaudio
import struct
import os

def main():
    # Initialize Porcupine with your custom keyword
    porcupine = pvporcupine.create(
        access_key=os.environ["PORCUPINE_ACCESS_KEY"],
        keyword_paths=["Hey-R-two_en_mac_v3_0_0.ppn"],
        sensitivities=[0.7]
    )

    pa = pyaudio.PyAudio()
    audio_stream = pa.open(
        rate=porcupine.sample_rate,
        channels=1,
        format=pyaudio.paInt16,
        input=True,
        frames_per_buffer=porcupine.frame_length
    )

    print("Listening for wake word...")

    try:
        while True:
            pcm = audio_stream.read(porcupine.frame_length, exception_on_overflow=False)
            pcm_unpacked = struct.unpack_from("h" * porcupine.frame_length, pcm)

            result = porcupine.process(pcm_unpacked)
            if result >= 0:
                print("✅ Wake word detected!")
                # Here you can trigger Whisper or your next command listener
                # For now just break or continue
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        audio_stream.stop_stream()
        audio_stream.close()
        pa.terminate()
        porcupine.delete()

if __name__ == "__main__":
    main()
