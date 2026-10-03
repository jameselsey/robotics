"""Import production dependencies and inspect installed resources without starting ROS."""

import importlib
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from pywebrtc_audio import AudioProcessor

for module in (
    "senses.voice_agent",
    "senses.joystick_voice_control",
    "senses.room_markers",
    "drive_controller.drive_controller_node",
    "drive_controller.angular_calibration",
    "strands.experimental.bidi.models",
    "pywebrtc_audio",
    "pyaudio",
    "gi",
    "openai",
    "websockets",
):
    importlib.import_module(module)

processor = AudioProcessor(
    sample_rate=16000,
    num_channels=1,
    echo_cancellation=True,
    noise_suppression=True,
    auto_gain_control=True,
    stream_delay_ms=10,
)
for filename in ("r2-sound-acknowledged.mp3", "stop-listening.mp3"):
    path = Path(get_package_share_directory("senses")) / "resource" / filename
    subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-f", "null", "-"], check=True)
share = Path(get_package_share_directory("tank_description"))
subprocess.run(["check_urdf", str(share / "urdf/robot.urdf")], check=True, capture_output=True)
subprocess.run(["ros2", "pkg", "executables", "sllidar_ros2"], check=True)
subprocess.run(
    ["ros2", "launch", "bringup", "all.launch.py", "--show-args"],
    check=True,
    stdout=subprocess.DEVNULL,
)
print("Production imports, AEC construction, installed resources and launch arguments pass.")
