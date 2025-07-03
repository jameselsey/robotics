# What is this?

This is a demo where I can play with `STT -> ollama -> TTS` in isolation.

Eventually, I'd like to be able to converse with my robot, so I'm using this to test that.

# How to use it?
Run `make install` to setup the environment

Run `make run` to start the demo, it'll sit there listening for the wake word

```
👂 Listening for wake word...
✅ Wake word detected!
||PaMacCore (AUHAL)|| Error on line 2523: err='-50', msg=Unknown Error
🎙️ Listening for command (auto-stop on silence)...
🔇 Silence detected, stopping recording.
🗣️ You said: What's the capital of Japan?
🤖 Sending to Ollama: What's the capital of Japan?
🧠 Ollama replied: 405 method not allowed


```


You might see `||PaMacCore (AUHAL)|| Error on line 2523: err='-50', msg=Unknown Error`, we can ignore this, it's a Mac error with PyAudio and doesn't seen to prevent it working.