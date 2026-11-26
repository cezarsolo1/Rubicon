# Rubicon

Quadruped robot ("the dog") with a RoArm-M2-S arm on the front, for landscaping and
construction tasks (see README.md).

## Stack and decisions
- Control: multi-process, iceoryx2 for IPC. **Prototype in Python first (P1, sim only), then
  migrate process by process to Rust (P2), then run on the robot (P3).**
- Sim: MuJoCo. Visualisation/recording: Foxglove (MCAP).
- Development is native (no containers). Fleet management later with balenaCloud (containers).
- Hardware: 12 × RobStride 00 motors over CAN (3 per leg: abduct, hip, knee), Bosch BNO055 IMU,
  Raspberry Pi onboard, camera on the base, RoArm-M2-S arm (ESP32, serial JSON).
- Walking: RL PPO policy, ONNX, 50 Hz, MIT (impedance) mode, trained **without** the arm.
  A policy trained with the arm comes later.
- Full requirements: `control/REQUIREMENTS.md` (draft, written together with the user;
  open questions in its last section).

## Repo layout
- `hardware/ro-arm/`: RoArm-M2 MuJoCo model (`ro-arm.xml`), URDF, meshes, `view.py`.
- `hardware/dog/`: articulated dog model. `build_dog.py` generates `dog.xml`, `meshes/` and
  `../dog_with_arm.xml` from `hardware/Full_dog.stl`. Generated files: edit the script, not the XML.
  `Full_dog.stl` was removed from the tree; restore with
  `git checkout 72e3c33 -- hardware/Full_dog.stl` before rebuilding.
- `hardware/dog_with_arm.xml`: dog + arm on the front plate; `hardware/view_dog_arm.py` opens it.
- `control/`, `app/`, `infra/`, `sim/`: mostly placeholders so far.

## Conventions
- Python: use the project venv `.venv/` (`.venv/bin/python ...`); it has mujoco and scipy.
- Model frames: front = +Y, left = −X, up = +Z. Joint names `{FL,FR,BL,BR}_{abduct,hip,knee}`
  and `arm_{base,shoulder,elbow,gripper}`.
- `dog_with_arm.xml` uses `angle="radian"`; angles in generated MJCF must be radians.
- Leg masses, joint limits and motor gains in `build_dog.py` are estimates; arm actuators use
  kp=60 in `ro-arm.xml`.

## Working with the user
- Write requirements and plans **together**: one topic at a time, ask the few decisions that
  matter, fold in the answers, explain the reasoning. Don't hand over a finished draft.

## Current status and next steps
Keep this section up to date at the end of each working session.

As of 2026-11-26:
- Done: arm model, articulated dog model, dog + arm scene (all viewable in MuJoCo);
  control requirements draft v0.4.
- Next, in order:
  1. Arm pressure/contact sensing: servo load feedback vs a force sensor at the gripper,
     and what force precision each task needs (plastering = hold a steady force,
     tiling = detect contact). Not yet decided.
  2. Interface spec: iceoryx2 services, message types, rates, mode state machine.
  3. Milestones M1–M6 (plumbing → stand → walk → arm → safety → first Rust process), then tickets.
- Still unknown: the PPO policy's observation/action definition (in the user's training code,
  not in this repo), CAN adapter and bus count, Pi model, BNO055 I²C vs UART, camera model.
- The Jira MCP server needs Docker, which isn't installed; tickets go in a markdown file for now.
