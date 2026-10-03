# Legacy Python environment

This is the temporary native ROS/Python workflow inherited from the Pi. It is
not the VENTUNO installation path. See [installation status](INSTALL.md) and the
[migration plan](VENTUNO_MIGRATION.md) before installing anything on the board.

The Makefile still creates `ros_venv` with system site packages so a native ROS 2
installation can provide `rclpy` and messages. The current `requirements.txt`
contains build helpers, PyAudio, NumPy, playsound, Strands/Nova dependencies,
Boto3, and the optional WebRTC audio processor. Pi GPIO still comes from the
native environment until the hardware adapter is replaced.

Whisper, Piper, Porcupine, and display-library dependencies were removed with
unused Pi experiments. The retained Nova agent does not import those libraries.
Wake-word inference runs in the separate Compose service, not in this venv.
Future local Whisper/Piper models will run in their own containers.

For the original Pi environment instructions and dependencies, use the
[baseline VENV_SETUP document](https://github.com/jameselsey/robotics/blob/8100627c087d5cc25e0c40bdf27b5d8a42e131bf/docs/VENV_SETUP.md)
and the source at that same revision.

The legacy environment targets remain until phase 3. Existing native deployments
can activate `ros_venv/bin/activate` and install the current `requirements.txt`
there. `make venv` only installs dependencies when creating a new environment;
it does not update one that already exists. Phase 3 will replace this workflow
with container builds and tests.
