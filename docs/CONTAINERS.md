# Container workflow on VENTUNO Q

Phase 3 provides the Linux build and service infrastructure. **The robot is still
not ready to drive:** phase 4 must implement local conversation and the STM32
adapter, finalize launch arguments, and verify firmware/pins. Both the Make
preflight and image launcher deliberately reject robot startup until then.
Do not bypass that gate with the old Pi launch commands.

## Build and check without hardware

Run from the repository root on this ARM64 board:

```bash
make config
make build
make test
make lint
```

These commands build images and run isolated checks; they do not launch robot
nodes or inference services. `checks` has no network, device mounts, credentials,
or Docker socket. It runs production import/AEC/MP3/URDF checks in a fresh Python
process, all package tests, deployment-policy tests, and incremental Ruff checks.
ROS, PyAudio, AWS/Strands, WebRTC, colcon, and the source-built LiDAR driver stay
inside images. No host ROS, venv, `pip install`, or `~/vendor_ws` is needed.

The ROS base and inference images are digest-pinned; `vendor.repos` pins the LiDAR
source commit. Python direct requirements are in `requirements.txt`; the complete
ARM64/CPython 3.12 hash lock is `docker/ros/requirements.lock`. NumPy stays at 1.26.4
for the Jazzy distribution ABI. Test tooling has a separate hash lock. Native
packages come from the Ubuntu/ROS apt repositories, so an uncached future build
can receive newer distribution patches; this is not a bit-for-bit apt snapshot.

The Dockerfile has dependency, Python, build, test, runtime, and RViz stages.
Runtime copies installed overlays rather than source/build trees. System layers
are shared with the build/test image, including compiler/test tools, to reuse the
existing Jazzy cache on this nearly full board. Runtime is not yet a minimal
production image. Allow at least 8 GiB free for a build without that cache.
The Makefile defaults to `DOCKER_BUILDKIT=0` to reuse the board's existing legacy
cache. Compose otherwise selects a separate BuildKit cache and can duplicate
several GiB of layers. On a host with sufficient space, opt in with
`DOCKER_BUILDKIT=1 make build`; the legacy builder is deprecated upstream.
No command automatically prunes Docker images or host files.

## Prepare persistent data

```bash
cp .env.example .env
# Review/edit .env: host UID/GID, paths, devices, ports; no secrets.
make prepare
```

`prepare` only creates project-owned `runtime/` directories and copies retained
map/custom wake-word files if the destination does not already exist. It never overwrites saved
maps or changes host device permissions. `.env` and `runtime/` are ignored by Git
and excluded from image build context. Back up `runtime/` separately.

| Data | Location |
| --- | --- |
| Maps saved by SLAM | `runtime/maps/` -> `/maps` |
| ROS home / localization / navigation journal | `runtime/state/ros/` -> `/state/ros` |
| ROS process logs | `runtime/state/logs/` -> `/state/logs` |
| Runtime home | `runtime/state/home/` -> `/state/home` |
| ASR/TTS models | `runtime/models/asr/`, `runtime/models/tts/` |
| Custom wake-word models | `runtime/models/wakeword/`, mounted read-only |
| ASR/TTS caches | Project-scoped `asr-cache` / `tts-cache` named volumes |
| GenieX model cache | `GENIEX_CACHE_DIR`, default `$HOME/.cache/geniex` |
| Drive/voice/room settings | Source YAML under `src/`, mounted read-only as `/config` |

The retained custom computer wake-word assets remain in Git and are copied to
runtime by `prepare`; their model ID is `computer` (the server strips `_v2`).
Large downloaded ASR/TTS/GenieX models stay outside Git. Application-specific
models are not baked into the ROS image. Download ASR/TTS with:

```bash
make setup-models
```

This needs internet and additional disk space. The downloader uses pinned
Whisper Small Quantized 0.57.0 and Piper English 0.57.1 packages with the
`qualcomm-qcs8275` model bundle names used by the verified board prototype.
Those are model artifact identifiers, not a claim that VENTUNO's MPU is QCS8275.
The device tree identifies this board as `arduino,monza` / QCS8300.
Downloads are explicit setup tasks, never part of ordinary robot startup.

To reuse existing models without copying them, set `ASR_MODELS_DIR` and
`TTS_MODELS_DIR` to their actual directories in `.env`. Defaults do not depend on
the neighboring voice checkout. Verify they contain complete model binaries and
no `.download` marker. `make prepare` does not download or copy models.

GenieX uses the board's existing NPU runtime, mounted read-only from
`GENIEX_RUNTIME_DIR` (default `$HOME/.local/share/geniex`) with updates disabled.
Preflight checks its executable against `docker/inference/geniex.sha256`.
FastRPC libraries and Qualcomm firmware/configuration remain host interfaces.
This preserves the working prototype's ABI without installing new host packages;
provisioning that runtime on another board remains a prerequisite. The cache
should contain `qualcomm/Qwen3-VL-4B-Instruct` or the reviewed `GENIEX_MODEL`.
A successful `/v1/models` response does not prove model warmup or tool support.

## Local inference only

```bash
python3 tools/preflight.py --mode local
make inference
```

This starts openWakeWord, separate Whisper/Piper vendor workers, and GenieX in
foreground. Ctrl-C stops this Compose invocation. It does not start ROS. Preflight rejects another running container using FastRPC devices. Do not
run a second inference stack against the same NPU/cache while the standalone
prototype is active; stop that prototype from its own terminal when deliberately
switching stacks. The agent does not stop it during migration.

Service APIs are loopback-only. Defaults use ports 18085 (ASR), 18086 (TTS), 28181
(GenieX), and 10400 (wake word), so the HTTP ports differ from the prototype.
Containers use Compose networking; the host-network ROS container accesses those
loopback-published APIs. Health checks mean API/TCP liveness, not completed model
initialization. Phase 4 will validate request/tool handling and readiness.

All services use `restart: "no"`; robot startup is always operator-controlled.
All services have explicit profiles, so plain `docker compose up` does not silently
start inference or hardware. Specify a service/profile intentionally. Do not use
`--profile '*' up`: it includes download tasks, tooling, and both ROS backends.

## Foreground robot workflow (enabled in phase 4)

The intended commands are:

```bash
make launch
make launch-navigation
make launch-localized MAP_NAME=house
make launch-nova
```

They select one ROS backend with `--abort-on-container-exit` and preserve Ctrl-C
control. Robot containers use SIGINT with a 30-second cleanup grace period and no
restart policy. Local ROS waits for inference/wake-word API health; Nova needs only
wake-word service. `ARGS='name:=value ...'` is forwarded as `ROS_LAUNCH_ARGS`, but
phase 4 must finalize its consumption alongside backend and hardware selection.
At phase 3 these commands fail preflight; they cannot yet drive or converse.

Only `compose.hardware.yaml` gives ROS access to camera, LiDAR, sound, input, and
the Router socket. It maps camera/LiDAR to stable container paths `/dev/video0`
and `/dev/ttyUSB0`; set their actual host paths or `/dev/*/by-id` links in `.env`.
Audio/input directories expose their device nodes; reconnecting a newly enumerated
USB device may require operator-controlled container recreation. There is no
blanket privileged mode or Pi GPIO device mapping.

Use the board's actual numeric audio/video/input/dialout/socket group IDs. The
sample IDs reflect this board, not every Linux machine. ROS runs as the configured
host UID/GID with added device groups, a read-only root filesystem, dropped
capabilities, and writable runtime mounts. Vendor accelerator workers run as root
inside their containers with specific FastRPC/DMA device mappings, as in the
prototype, without changing device permissions on the host.

Nova credentials are mounted read-only **only** by `compose.nova.yaml` into
`ros-nova`. Set `AWS_DIR` to an existing shared-profile directory, with
`AWS_PROFILE` / `AWS_REGION` as non-secret configuration. Credentials never enter
images, YAML, or local ROS. Shared credential/config profiles are supported by
this mount; SSO token refresh/cache behavior needs validation in phase 4.

## Diagnostics and calibration

```bash
make status
make logs
make health
make shell
make save-map MAP_NAME=house
make navigation-log
make calibrate-angular
```

ROS commands execute in the existing container after sourcing its overlays; they
never create another robot process. Use `ROS_SERVICE=ros-nova` for Nova diagnostics.
`save-map` writes persistent `/maps`; it does not change SLAM lifecycle state.
Angular calibration remains passive and does not rewrite drive YAML. Review its
printed result, update the source config yourself, and relaunch from your terminal.
No diagnostic command makes robot startup ready in phase 3.

When the full robot is running after later phases, Foxglove on the Mac connects to
`ws://<VENTUNO-hostname-or-IP>:8765` (or `FOXGLOVE_PORT`). Inference ports stay local.

## Optional RViz

```bash
make build-rviz
make rviz
```

RViz now uses the ARM64 Jazzy base and the same DDS implementation/domain as ROS.
The hardcoded `robopi` CycloneDDS peer and amd64-only desktop base were removed.
This GUI workflow is for a Linux host with an accessible X display; set `DISPLAY`
and arrange X authorization yourself. Docker Desktop/XQuartz networking on a Mac
is not verified. Use Foxglove from the Mac for the planned robot workflow.

## Updating dependencies

Review `requirements.txt`, resolve it in a disposable ARM64/Python 3.12 container
with `pip install --dry-run --ignore-installed --report report.json -r requirements.txt`,
then run `python3 tools/lock-python.py report.json docker/ros/requirements.lock`.
Review the changed versions/hashes, rebuild, and run `make test` before accepting.
Use the corresponding disposable build stage when regenerating the lock; do not
install application dependencies into host Python. Update vendor SHAs and image
digests explicitly and validate them; never replace immutable pins with `latest`.
