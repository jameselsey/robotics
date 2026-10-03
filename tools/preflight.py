#!/usr/bin/env python3
"""Read-only deployment checks using Compose's resolved paths and environment."""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def check(mode):
    args = ["docker", "compose", "-f", str(ROOT / "docker-compose.yml")]
    if mode in {"robot", "nova"}:
        args += ["-f", str(ROOT / "compose.hardware.yaml")]
    if mode == "nova":
        args += ["-f", str(ROOT / "compose.nova.yaml")]
    args += ["--profile", "*", "config", "--format", "json"]
    config = json.loads(subprocess.check_output(args, cwd=ROOT, text=True))
    errors = []
    if mode == "build":
        available = shutil.disk_usage(ROOT).free
        print(
            f"Free disk: {available / 2**30:.1f} GiB; allow at least 8 GiB for an uncached build."
        )
        return errors
    selected = (
        ["ros-nova", "openwakeword"]
        if mode == "nova"
        else ["speech", "tts", "geniex", "openwakeword"]
    )
    if mode == "robot":
        selected.append("ros")
    for name in selected:
        service = config["services"][name]
        for volume in service.get("volumes", []):
            if volume["type"] == "bind" and not Path(volume["source"]).exists():
                errors.append(f"{name}: missing {volume['source']}")
        for device in service.get("devices", []):
            source = device.split(":", 1)[0] if isinstance(device, str) else device["source"]
            if not Path(source).exists():
                errors.append(f"{name}: missing device {source}")
    wake_mounts = config["services"]["openwakeword"].get("volumes", [])
    custom = next((Path(v["source"]) for v in wake_mounts if v["target"] == "/custom"), None)
    if custom is not None and not list(custom.glob("*.tflite")):
        errors.append(f"Wake-word TFLite models missing under {custom}; run make prepare")
    if mode != "nova":
        for name, folder in (("speech", "asr"), ("tts", "tts")):
            mounts = config["services"][name]["volumes"]
            source = Path(
                next(v["source"] for v in mounts if v["target"] == f"/mnt/work/models/{folder}")
            )
            if not list(source.glob("**/*.bin")):
                errors.append(f"{name}: model binaries missing under {source}; run model setup")
            if list(source.glob("**/.download")):
                errors.append(f"{name}: incomplete model download under {source}")
        mounts = config["services"]["geniex"]["volumes"]
        runtime = Path(next(v["source"] for v in mounts if v["target"] == "/opt/geniex"))
        binary = runtime / "geniex"
        if binary.is_file():
            expected = (ROOT / "docker/inference/geniex.sha256").read_text().split()[0]
            with binary.open("rb") as stream:
                actual = hashlib.file_digest(stream, "sha256").hexdigest()
            if actual != expected:
                errors.append(
                    "GenieX binary differs from the validated runtime; review before updating the checksum"
                )
        else:
            errors.append(f"GenieX executable missing: {binary}")
    if mode != "nova":
        running = subprocess.check_output(["docker", "ps", "-q"], text=True).split()
        if running:
            containers = json.loads(
                subprocess.check_output(["docker", "inspect", *running], text=True)
            )
            for container in containers:
                project = (container["Config"].get("Labels") or {}).get(
                    "com.docker.compose.project"
                )
                devices = container["HostConfig"].get("Devices") or []
                if project != config["name"] and any("fastrpc" in d["PathOnHost"] for d in devices):
                    errors.append(
                        f"Accelerator already used by {container['Name'].lstrip('/')}; switch stacks from its own terminal"
                    )
    if mode in {"robot", "nova"}:
        errors.append("Robot startup remains gated until phase 4 MCU/voice integration is complete")
    return errors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("build", "local", "robot", "nova"), default="build")
    options = parser.parse_args()
    failures = check(options.mode)
    for failure in failures:
        print(failure, file=sys.stderr)
    if failures:
        sys.exit(1)
    print("Preflight passed; API liveness still does not establish model readiness.")
