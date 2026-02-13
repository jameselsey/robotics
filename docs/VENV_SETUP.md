# Virtual Environment Setup

This project uses a Python virtual environment to manage dependencies for the `senses` package, avoiding conflicts with system packages.

**IMPORTANT**: This setup is designed for the Raspberry Pi/robot. The venv uses `--system-site-packages` to inherit ROS2 packages from the system while adding ML/AI libraries on top.

## Why Use a Venv?

The `senses` package uses heavy ML/AI libraries (Whisper, Piper TTS) that have complex dependency trees. Installing these via `rosdep` can cause conflicts with system packages (like the `zipp` issue). Using a venv isolates these dependencies while still allowing ROS2 system integration.

## Quick Start (On the Robot)

The Makefile handles everything automatically:

```bash
# Create venv and install dependencies
make venv

# Install ROS2 system dependencies
make install-deps

# Build (automatically uses venv)
make build

# Launch (automatically uses venv)
make launch-senses
```

## Manual Setup (On the Robot)

If you need to manually work with the venv:

```bash
# Activate the venv
source ros_venv/bin/activate

# Install/update dependencies
pip install -r requirements.txt

# Deactivate when done
deactivate
```

## What's Managed Where?

### Via Venv (requirements.txt)
- openai-whisper
- piper-tts
- pyaudio, soundfile, sounddevice
- numpy, onnxruntime
- requests
- librosa

### Via rosdep (package.xml)
- rclpy and ROS2 packages
- System audio libraries (portaudio, libsndfile)
- GPIO libraries (gpiozero)

## Troubleshooting

### "Module not found" errors
Make sure the venv is activated:
```bash
source ros_venv/bin/activate
```

### Reinstall dependencies
```bash
make clean-venv
make venv
```

### Add new Python packages
1. Add to `requirements.txt`
2. Run `pip install -r requirements.txt` (with venv activated)
3. Or run `make venv` to recreate

## OpenWakeWord

OpenWakeWord is installed separately via:
```bash
make install-oww
```

This installs it into the venv without pulling in unnecessary dependencies.
