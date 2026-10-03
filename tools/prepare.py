#!/usr/bin/env python3
"""Create project-owned persistent directories; never alter device permissions."""

import shutil
from pathlib import Path

root = Path(__file__).resolve().parents[1]
for relative in (
    "state/home",
    "state/ros",
    "state/logs",
    "maps",
    "models/asr",
    "models/tts",
    "models/wakeword",
    "results",
):
    (root / "runtime" / relative).mkdir(parents=True, exist_ok=True)

for source in (root / "maps").glob("*"):
    target = root / "runtime/maps" / source.name
    if source.is_file() and source.suffix in {".yaml", ".pgm", ".png"} and not target.exists():
        shutil.copy2(source, target)

for source in (root / "openwakeword/custom").glob("*"):
    target = root / "runtime/models/wakeword" / source.name
    if source.is_file() and source.suffix in {".tflite", ".onnx"} and not target.exists():
        shutil.copy2(source, target)

print("Project runtime directories prepared. Review .env.example before copying to .env.")
