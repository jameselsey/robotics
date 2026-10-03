"""The unfinished MCU migration must fail before old GPIO nodes can start."""

import importlib.util
from pathlib import Path

import pytest


def test_phase3_launcher_is_gated_for_both_backends(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "docker/ros/robot-start.py"
    spec = importlib.util.spec_from_file_location("robot_start", path)
    launcher = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launcher)
    looked_up = []

    def absent(name):
        looked_up.append(name)
        return None

    monkeypatch.setattr(launcher.importlib.util, "find_spec", absent)
    for backend in ("local", "nova"):
        monkeypatch.setenv("VOICE_BACKEND", backend)
        with pytest.raises(SystemExit, match="phase 4 adapters"):
            launcher.main()
    assert looked_up == [
        "drive_controller.mcu_backend",
        "senses.local_backend",
        "drive_controller.mcu_backend",
    ]
    monkeypatch.setenv("VOICE_BACKEND", "invalid")
    with pytest.raises(SystemExit, match="Unsupported VOICE_BACKEND"):
        launcher.main()
