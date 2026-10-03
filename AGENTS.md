# RoboPi operating rule

- Never start, stop, restart, or relaunch the robot's ROS 2/robotics launch processes.
- Code changes, builds, tests, and read-only runtime diagnostics over SSH are allowed.
- When changes require a relaunch, tell the user and let them run it from their own SSH terminal so they retain Ctrl-C control.

# VENTUNO migration workflow

- Before migration work, read `docs/VENTUNO_MIGRATION.md` for the agreed plan, decisions, current phase, validation results, and next step. Check the working tree and Git history against that record before continuing.
- Work on `main`, one numbered phase at a time. Stop at each completed phase for user review; do not begin the next phase until requested.
- The user owns commits, tags, and pushes. Do not create commits, tags, or branches, or push changes unless explicitly instructed.
- Update the migration document at each checkpoint and before pausing unfinished work. Distinguish implementation complete, awaiting review, committed, and physically verified; do not infer user approval or a commit from elapsed time.
- The VENTUNO is not yet wired to the robot. Use hardware-independent tests, fixtures, and simulation; report physical acceptance as pending.
- Query the actual board and installed CLI/library sources before board-dependent choices. Never reuse Raspberry Pi BCM numbers or an UNO Q firmware target for VENTUNO.
- Verify pin capabilities and electrical requirements before documenting wiring assignments. Mark unresolved assignments as pending instead of inventing them.
- Keep new application dependencies in containers or disposable test environments. Do not remove unrelated host packages, prune Docker storage, alter device permissions, or change board services as part of repository cleanup.
- Preserve existing ROS topics, actions, TF frames, calibration, navigation safeguards, maps, and Foxglove functionality unless the user agrees to a change.
