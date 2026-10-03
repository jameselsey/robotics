# VENTUNO Q migration plan and progress

This is the durable handoff for the migration from Raspberry Pi 5 to Arduino
VENTUNO Q. Read it alongside `AGENTS.md`, the working tree, and Git history when
resuming in a new session. Update it at phase checkpoints and before pausing
unfinished work. The agreed plan originated on 2026-10-03.

## Current checkpoint

- **Active phase:** 1, repository preservation and cleanup.
- **Implementation:** phase 1 cleanup complete and verified; ready for review.
- **User review / commit / push:** pending. No commits, tags, or pushes made by the agent.
- **Next step:** user reviews the diff, creates/pushes the baseline tag, and
  commits/pushes the cleanup. Begin phase 2 only when explicitly requested.
- **Hardware:** the VENTUNO has not been installed on the chassis or wired to its
  motors, encoders, LED, LiDAR, controller, or robot audio devices. Physical
  acceptance remains pending even if individual peripherals appear on the board.

| Phase | Status | Checkpoint |
| --- | --- | --- |
| 1. Preserve and clean | Implementation complete | User review, baseline tag, and commit/push pending |
| 2. ROS packaging/build | Not started | Wait for an explicit request after phase 1 review |
| 3. Compose infrastructure | Not started | Clean container build and operator workflow |
| 4. Local voice and MCU | Not started | Mock integration, firmware build, and verified wiring guide |
| 5. Integration and handoff | Not started | Software acceptance; physical checks performed by user |

## Working agreement

- Continue on `main`; no separate VENTUNO development branch or Pi maintenance branch.
- Work one numbered phase at a time. At each checkpoint, summarize changes,
  validation, pending work, and a suggested commit message, then stop for review.
- The user owns commits, tags, and pushes. Never infer that review or publishing
  happened; inspect Git and ask only if the next requested task needs clarification.
- Never start, stop, restart, or relaunch the robot's ROS/robotics processes.
  The user launches from their own terminal and retains Ctrl-C control.
- Use isolated tests and simulation while hardware is disconnected. Never label
  software checks as physical verification.
- No unrelated host cleanup, Docker pruning, permission changes, or board-service
  changes. Install new application dependencies in images or disposable environments.
- Preserve ROS topic/action/frame contracts, calibration tools, navigation
  safeguards, maps, and useful project assets.

## Preserve the Pi baseline

The agreed Pi baseline is **`8100627c087d5cc25e0c40bdf27b5d8a42e131bf`**
(`Add angular odometry calibration tool`). The local working tree was clean and
GitHub `main` matched this commit during the planning inventory.

The agent has not created the archive tag. The user should run these commands
before committing the cleanup; the explicit SHA still selects the original Pi
code if `main` has moved:

```bash
git tag -a pi5-final 8100627c087d5cc25e0c40bdf27b5d8a42e131bf -m "Final Raspberry Pi 5 baseline before VENTUNO Q migration"
git show --no-patch pi5-final
git push origin pi5-final
```

If the tag already exists, inspect its target instead of overwriting it. Keep
the immutable [Pi baseline](https://github.com/jameselsey/robotics/tree/8100627c087d5cc25e0c40bdf27b5d8a42e131bf)
link usable regardless of tag publication. The original install instructions,
experiments, and models remain available at that revision; do not rewrite history.

## End-to-end acceptance target

After all phases and physical wiring, the user must be able to:

- Start the robot using the documented foreground Compose workflow.
- Connect Foxglove from a Mac on the same network to the robot's port 8765.
- See the previously available camera, LiDAR, map, robot description, odometry,
  transforms, room/route visualization, voice information, and diagnostics.
- Connect an Xbox controller, use deadman driving, and control voice sessions.
- Use measured encoder odometry, SLAM mapping, saved-map localization, semantic
  room annotations, route preview, and Nav2 navigation with existing safeguards.
- Talk to the local voice agent, hear responses, ask camera questions, and use
  validated robot tools. Select Nova Sonic explicitly when desired.
- Observe motors stopping on command expiry, communication loss, reset, and
  shutdown; reconnect must not replay a stale command.

## Agreed architecture and defaults

- **ROS:** keep all ROS nodes and launch orchestration in one ARM64 Jazzy/Noble
  container, including audio capture/playback, conversation control, and robot tools.
- **Services:** separate openWakeWord, GenieX, Whisper ASR, Piper TTS, and optional
  RViz containers. Models and runtime state live outside images and Git.
- **Voice:** local Whisper -> GenieX -> Piper by default; retain selectable Nova.
  Local conversation is turn-taking, with input paused during responses.
  Preserve joystick cancellation; defer acoustic echo cancellation/barge-in for
  local voice. Retain the existing optional Nova audio processor.
- **Reference:** `/home/arduino/dev/voice` contains an existing local inference
  prototype. Adapt its service definitions and API clients into this repository;
  do not require that neighboring checkout to operate the finished robot.
- **Hardware:** retain the existing motors/drivers, encoders, LED, USB LiDAR,
  camera/microphone, speakers, and Xbox controller. Put PWM, quadrature counting,
  and LED output on STM32; retain ROS command mixing and odometry on Linux.
- **Transport:** use Arduino Router Bridge and the host Router service/socket.
  Firmware provisioning is separate from normal Compose startup.
- **Host boundary:** kernel drivers, device permissions, Docker, Arduino Router,
  and board accelerator firmware/runtime interfaces remain host responsibilities.
  ROS, Python libraries, and source-built application dependencies belong in containers.
- **Networking:** ROS uses host networking for DDS and Foxglove. Inference uses
  Compose networking and loopback-published APIs reachable by the ROS container.
- **Startup:** manual foreground robot startup, no automatic robot restart policy.

## Phased implementation

### Phase 1 — Preserve and clean the repository

- Document the Pi baseline and tag handoff prominently in the root README.
- Remove unused old voice/display experiments, logging-only camera node,
  duplicated models/assets, Ollama, and commented Hailo configuration.
- Remove their entry points and direct dependencies together. Preserve both
  active voice sound effects, camera capture/throttle, and existing robot logic.
- Consolidate outdated install, environment, camera, and mapping documentation;
  mark historical Pi instructions and power notes clearly.
- Persist this plan and future-session operating instructions in `AGENTS.md`.

### Phase 2 — ROS packaging and build cleanup

- Retain valid `ament_python` executable packages and `ament_cmake` resource
  packages. Correct metadata, manifests, installation, and direct dependencies.
- Separate responsibilities inside `senses` without renaming packages or public
  ROS interfaces. Preserve calculations, calibration, and navigation safeguards.
- Generate the canonical Xacro's URDF in build/install output; update lookup and
  tests so builds no longer overwrite tracked source. Review unused description
  resources and the empty view launch file as part of this build cleanup.
- Parameterize deployment paths, hostnames, devices, and settings. Introduce
  focused behavior tests and incremental style checks.

### Phase 3 — Reproducible Compose infrastructure

- Build ARM64 Jazzy/Noble images with build/test and runtime stages. Pin tested
  images/digests and Python dependencies.
- Build LiDAR/vendor code from a checked-in manifest with immutable revisions;
  remove reliance on `~/vendor_ws`, host ROS installation, and host venvs.
- Integrate separate inference/wake-word services and optional RViz tooling.
  Configure device passthrough without blanket privileged access.
- Persist maps, localization state, logs, downloads, and caches. Mount cloud
  credentials read-only only for the Nova option, never in source or ROS parameters.
- Replace legacy Make targets with container build/test/diagnostic and foreground
  launch workflows, preserving mapping/localized modes and calibration operations.

### Phase 4 — Local voice and STM32 integration

- Reuse persistent ASR and streamed LLM/sentence playback from the prototype,
  with bounded queues, cancellation, timeouts, and failure diagnostics.
- Preserve wake words, joystick/session control, transcripts, voice state, vision,
  movement limits, and localization/navigation protections across backends.
- Abstract inference and LED/hardware access. Test structured GenieX tools with
  harmless mocks before enabling motion tools; unsupported tools fail closed.
- Implement simulated and real Bridge adapters; put PWM, quadrature counting,
  and LED output in firmware with an independent command-expiry watchdog.
- Handle disconnects, resets, malformed/stale commands, and encoder discontinuity.
- Verify the VENTUNO firmware target, pin capabilities, and electrical requirements
  before assigning wires or flashing. Never use an UNO Q target or Pi BCM numbers.
- Document connector orientation, physical pin numbers, PWM/interrupt capability,
  ground and power, logic levels, driver-enable wiring, firmware setup, and initial
  checks. Mark unresolved facts pending; physical deployment is user-operated.

### Phase 5 — Integration, CI, and deployment handoff

- Add clean build/config/package and hardware-independent CI tests.
- Validate simulated ROS modes, topic/TF contracts, persistence, shutdown,
  inference readiness, cancellation, and reconnection. Use recorded fixtures where available.
- Document Mac/Foxglove access and the foreground startup/diagnostic workflow.
- User performs staged acceptance: USB devices, LED, hand-turned encoders,
  wheels-raised motors/watchdogs, odometry recalibration, manual driving,
  mapping/localization/navigation, then voice tools.
- Track physical checks as pending until observed; fix hardware-specific findings
  without silently changing the agreed coordinate or ROS contracts.

## Evidence and known gates

The following were observed during planning and phase 1; query again before using
them for a board-dependent implementation choice:

- Device-tree `arduino,monza` identifies VENTUNO Q. OS: Ubuntu 24.04.4 ARM64.
- Docker Engine 29.1.3, Compose 2.24.6, Arduino App CLI/daemon 0.12.1.
- Router socket exists at `/var/run/arduino-router.sock`.
- Installed Arduino core 0.90.0 exposes only `arduino:zephyr:unoq` in the board
  catalog. A correct VENTUNO MCU toolchain/target must be verified before firmware work.
- Host ROS and pytest were absent. Do not install ROS on the host to bypass this.
- Existing motor code has no command-expiry watchdog; MCU migration must add one.
- The local inference prototype documents successful short tests, not production
  robot operation. API health does not prove model readiness or tool support.
- GenieX QAIRT tool calling needs explicit validation on this board/model; an
  [upstream report](https://github.com/qualcomm/GenieX/issues/1454) describes tools
  missing from rendered prompts on another deployment. Treat it as a validation
  gate, not proof that this installation fails.

## Phase 1 change and validation record

Implementation changes:

- Removed the standalone `whisper/` demo and old `ears`, `mouth`, `brain`,
  `screen`, and `led` modules. Removed `eyes`, which only logged startup.
- Removed legacy console entry points and node-local Whisper/Piper/Porcupine,
  unused audio/ML, display, and in-process wake-word dependencies.
- Removed duplicated asset copies and unused Piper/Porcupine models. Kept
  `r2-sound-acknowledged.mp3` and `stop-listening.mp3` in the installed resource directory.
- Removed Ollama/Hailo Compose references, the Hailo shell target, and unused
  in-process wake-word install target. Retained openWakeWord and RViz services.
- Updated documentation and removed misleading fake-odometry/package claims,
  centroid-navigation advice, and outdated static-transform ownership.

Validation before cleanup: **49 passed, 1 deselected**, covering kinematics,
movement/navigation protections, semantic rooms, voice activity, syntax, and
robot-description geometry. Tests used temporary pytest/PyYAML packages in
`/tmp/robotics-phase1-test-deps`; no application packages were installed into host Python.

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/tmp/robotics-phase1-test-deps:src/drive_controller:src/senses \
  python3 -m pytest -p no:cacheprovider -q \
  src/drive_controller/test src/senses/test \
  src/tank_description/test/test_robot_description.py -k 'not generated_urdf'
```

The deselected test needs `xacro` and `check_urdf`, which are absent here. Full
colcon builds, launch tests, MCU compilation, voice inference, browser UI checks,
and all physical tests are pending later phases. No robot or existing inference
services were started, stopped, or restarted during phase 1.

Final verification on 2026-10-03:

- **49 passed, 1 deselected** after cleanup, matching the pre-cleanup baseline.
  The description-toolchain test remains pending for the ROS build image.
- All **34 retained Python files** parse; all **5 ROS package manifests** parse.
- Every retained console entry point resolves to an existing module/function;
  local launch executables resolve to retained entry points or installed scripts.
- No retained Python imports target removed modules.
- Both active sound effects match the baseline byte-for-byte. Core drive, Nova
  voice/LED, joystick launch, full bringup, and senses launch also match the baseline.
  Camera capture/throttling remains; only its logging-only custom node was removed.
- `docker compose config --quiet` passes; the tools profile lists only
  `openwakeword` and `rviz`. No service was launched.
- Make targets were inspected with dry runs; no build or launch recipes executed.
- `git diff --check` passes. Internal Markdown file/image links were checked.
- Approximately 120 MiB of unused model copies were removed from the checkout;
  those files remain available in Git history.

The isolated test runner was pytest 9.1.1 with PyYAML 6.0.3, installed only under
`/tmp/robotics-phase1-test-deps`. That temporary directory is not needed by the
application or committed to Git; recreate test tooling in a disposable environment
after reboot if necessary. Containerized test setup is part of phases 2/3.

No remaining application dependency removal or node restructuring is included
in phase 1. Manifest completion, reproducible builds, generated description
cleanup, MCU support, and local inference remain in their numbered phases.

Suggested phase 1 commit message: `Preserve Pi baseline and clean obsolete robotics experiments`.
