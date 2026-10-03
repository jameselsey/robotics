#!/usr/bin/env python3
"""Foreground robot launcher. Phase 4 must supply the selected hardware/voice adapters."""

import importlib.util
import os
import sys


def main():
    backend = os.environ.get("VOICE_BACKEND", "local")
    # Fail before importing legacy nodes or opening hardware. These modules are
    # deliberately absent in phase 3; phase 4 will implement/validate the contract.
    required = ["drive_controller.mcu_backend"]
    if backend == "local":
        required.append("senses.local_backend")
    elif backend != "nova":
        sys.exit(f"Unsupported VOICE_BACKEND: {backend}")
    missing = [module for module in required if importlib.util.find_spec(module) is None]
    if missing:
        sys.exit("Robot startup requires phase 4 adapters: " + ", ".join(missing))
    sys.exit(
        "Phase 4 must finalize launch arguments and MCU readiness checks before robot startup."
    )


if __name__ == "__main__":
    main()
