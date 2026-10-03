"""Deployment invariants: isolation, opt-in credentials, and persistent artifacts."""

import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_robot_is_foreground_and_devices_are_scoped():
    config = yaml.safe_load((ROOT / "docker-compose.yml").read_text())
    services = config["services"]
    for service in services.values():
        assert service.get("privileged", False) is False
        assert service.get("restart", "no") == "no"
        assert service.get("profiles")  # plain `up` must not unexpectedly start hardware
    for name in ("ros", "ros-nova"):
        robot = services[name]
        assert robot["stop_signal"] == "SIGINT"
        assert robot["network_mode"] == "host"
        assert robot["read_only"] and "devices" not in robot
        assert not any(".aws" in str(v) for v in robot["volumes"])
    assert services["checks"]["network_mode"] == "none"
    assert "volumes" not in services["checks"] and "devices" not in services["checks"]
    for name in ("speech", "tts", "geniex", "openwakeword"):
        assert all(port.startswith("127.0.0.1:") for port in services[name]["ports"])
        assert "@sha256:" in services[name]["image"]
    nova = yaml.safe_load((ROOT / "compose.nova.yaml").read_text())["services"]
    assert list(nova) == ["ros-nova"]
    assert nova["ros-nova"]["volumes"][0]["read_only"] is True
    hardware = yaml.safe_load((ROOT / "compose.hardware.yaml").read_text())["services"]
    assert set(hardware) == {"ros", "ros-nova"}
    assert len(hardware["ros"]["devices"]) == 4
    assert hardware["ros"]["volumes"][0]["bind"]["create_host_path"] is False


def test_vendor_and_python_dependencies_are_locked():
    vendors = yaml.safe_load((ROOT / "vendor.repos").read_text())["repositories"]
    assert set(vendors) == {"sllidar_ros2"}
    assert re.fullmatch(r"[a-f0-9]{40}", vendors["sllidar_ros2"]["version"])
    lock = (ROOT / "docker/ros/requirements.lock").read_text()
    packages = [line for line in lock.splitlines() if line and not line.startswith("#")]
    assert len(packages) > 20
    assert all("==" in line and "--hash=sha256:" in line for line in packages)
    ignore = (ROOT / ".dockerignore").read_text().splitlines()
    assert {".git", ".aws", ".env", "runtime", "ros_venv"}.issubset(ignore)
