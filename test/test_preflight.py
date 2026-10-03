"""Read-only preflight handles Compose device formats and rejects accelerator conflicts."""

import hashlib
import importlib.util
import json
from pathlib import Path


def test_preflight_checks_models_device_formats_and_other_projects(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[1] / "tools/preflight.py"
    spec = importlib.util.spec_from_file_location("preflight", path)
    preflight = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preflight)
    monkeypatch.setattr(preflight, "ROOT", tmp_path)
    runtime = tmp_path / "geniex"
    runtime.mkdir()
    binary = runtime / "geniex"
    binary.write_bytes(b"fixture-runtime")
    checksum = tmp_path / "docker/inference/geniex.sha256"
    checksum.parent.mkdir(parents=True)
    checksum.write_text(hashlib.sha256(binary.read_bytes()).hexdigest())
    services = {"openwakeword": {}}
    for name, folder in (("speech", "asr"), ("tts", "tts")):
        model = tmp_path / folder
        model.mkdir()
        (model / "encoder.bin").write_bytes(b"fixture-model")
        services[name] = {
            "volumes": [
                {"type": "bind", "source": str(model), "target": f"/mnt/work/models/{folder}"}
            ],
        }
    services["geniex"] = {
        "volumes": [{"type": "bind", "source": str(runtime), "target": "/opt/geniex"}],
    }
    config = {"name": "robotics", "services": services}
    running = []

    def output(args, **kwargs):
        if args[1] == "compose":
            assert args[-2:] == ["--format", "json"]
            return json.dumps(config)
        if args[1] == "ps":
            return "" if not running else "container-id"
        assert args[1] == "inspect"
        return json.dumps(running)

    monkeypatch.setattr(preflight.subprocess, "check_output", output)
    assert preflight.check("local") == []
    services["speech"]["devices"] = ["/missing-fixture-device:/container-device:rwm"]
    services["tts"]["devices"] = [{"source": "/other-missing-device"}]
    (tmp_path / "asr/.download").touch()
    running.append(
        {
            "Name": "/another-stack-worker",
            "Config": {"Labels": {"com.docker.compose.project": "another-stack"}},
            "HostConfig": {"Devices": [{"PathOnHost": "/dev/fastrpc-cdsp"}]},
        }
    )
    errors = preflight.check("local")
    assert len(errors) == 4
    assert any("missing-fixture-device" in error for error in errors)
    assert any("other-missing-device" in error for error in errors)
    assert any("incomplete model download" in error for error in errors)
    assert any("another-stack-worker" in error for error in errors)
