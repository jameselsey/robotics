#!/usr/bin/env python3
"""Convert a reviewed ARM64/Python 3.12 pip --report resolution into a hash lock."""

import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text())
lines = [
    "# ARM64 / CPython 3.12 application dependencies; see requirements.txt.",
    "# Generated from pip --dry-run --ignore-installed --report; distribution packages remain apt-owned.",
]
for entry in sorted(report["install"], key=lambda item: item["metadata"]["name"].lower()):
    name, version = entry["metadata"]["name"], entry["metadata"]["version"]
    digest = entry["download_info"]["archive_info"]["hashes"]["sha256"]
    lines.append(f"{name}=={version} --hash=sha256:{digest}")
Path(sys.argv[2]).write_text("\n".join(lines) + "\n")
