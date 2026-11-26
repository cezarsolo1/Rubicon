# Rubicon control pipeline: requirements (draft v0.4)

Status: **draft, being written together**. Items marked **[OPEN]** still need a decision (§10).
Numbers marked *(est.)* are first estimates to be confirmed on hardware or from datasheets.

## 1. Purpose and scope

A multi-process control stack for the Rubicon quadruped and its RoArm-M2-S manipulator.
Each component (motors, sensors, policy, arm, safety, visualisation) runs as its own process,
and the processes exchange data over **iceoryx2** shared memory.

In scope: walking with a trained RL (PPO) policy, arm control with contact sensing,
camera, safety supervisor, teleoperation, visualisation and recording with Foxglove, and
running the same stack against the MuJoCo simulation.
Out of scope for now: autonomous navigation, the application tools (painting, plastering,
mowing, …), and the TypeScript app's own features (it connects through the gateway, §6.8).

### Development phases

| Phase | Language | Runs against | Goal |
|---|---|---|---|
| **P1: prototype** | Python | MuJoCo sim only | Full stack works in sim: stand, sit, **walk with the PPO policy**, arm moves and detects contact |
| **P2: migration** | Rust, one process at a time, alongside the remaining Python processes | Sim, then hardware | Meet the real-time requirements (§5) |
| **P3: hardware** | Rust for the real-time path; Python allowed for non-real-time tools | Robot | Run on the Raspberry Pi |

- **PH-1** Every message type is defined once in a language-neutral schema, and both the
  Python `ctypes` structs and the Rust `#[repr(C)]` structs are generated from it, so the
  two layouts cannot drift apart. Schema format **[OPEN]**.
- **PH-2** Mixed Python/Rust operation is required: any single process can be swapped from
  Python to Rust without changing the others. Service names, payload type names, sizes and
  alignment must match across both languages.
- **PH-3** A cross-language test publishes each message type from Python and receives it
  in Rust (and the reverse), and checks the values match field by field.
- **PH-4** Python processes keep the same process boundaries, service names and config files
  as the Rust ones will use, so that migrating only replaces the implementation.
- **PH-5** P1 code avoids patterns that do not translate to Rust: no dynamically-shaped
  messages, no shared global state between processes, no pickling over IPC.
- **PH-6 (P1 exit criteria)** In sim, under teleop: the dog goes passive → stand → walk
  (forward/back/turn on command) → stand → sit; the arm reaches a commanded target while
  standing and stops when it touches an obstacle; everything is visible live in Foxglove
  and recorded to a file. The goal of P1 is a working pipeline and an understanding of it,
  not walking performance: walking with the current policy only needs to work well enough
  to exercise every process and message.

## 2. System context

| Component | Hardware | Interface | Notes |
|---|---|---|---|
| Legs | 12 × **RobStride 00** (3 per leg: abduction, hip, knee) | CAN, 1 Mbit/s | Torque/speed limits from the datasheet *(to fill in)* |
| CAN adapter | **[OPEN]** | USB or SPI HAT | The Pi has no built-in CAN (§10 risk R-2) |
| Arm | Waveshare RoArm-M2-S (4 servos, ESP32) | USB serial (JSON) or Wi-Fi **[OPEN]** | Low rate, low torque |
| Arm contact sensing | **[OPEN]**: servo load feedback and/or a force sensor at the gripper | — | §6.4 |
| IMU | **Bosch BNO055** (GY-BNO055 board) | I²C or UART **[OPEN]** | Fused orientation output up to 100 Hz (§10 risk R-3) |
| Camera | **[OPEN]** model, on the robot base | USB or CSI | §6.6 |
| Onboard computer | **Raspberry Pi** (model **[OPEN]**) | — | Linux; real-time kernel **[OPEN]** |
| Simulation | MuJoCo: `hardware/dog_with_arm.xml` | Python bindings | Stands in for all hardware |

Joint naming follows the sim model: `{FL,FR,BL,BR}_{abduct,hip,knee}` and
`arm_{base,shoulder,elbow,gripper}`. Front = +Y, left = −X.

## 3. Architecture requirements

- **AR-1** Each component runs as a separate OS process. If a non-critical process crashes,
  the rest keep running and the robot goes to a safe state (§7).
- **AR-2** All inter-process data goes over iceoryx2, using publish/subscribe for streams,
  request/response for commands and configuration, and events for wake-ups.
- **AR-3** Messages are fixed-size, self-contained types (no heap pointers) so they can be
  sent zero-copy. Variable-size data (camera images) uses fixed-capacity slices.
- **AR-4** Every message carries a monotonic timestamp (ns) and a sequence number.
- **AR-5** Hardware and simulation are interchangeable: `sim_bridge` publishes and
  subscribes to exactly the same services as the hardware bridges. Controllers must not
  know which one is running.
- **AR-6** Service names are versioned (e.g. `rubicon/v1/legs/state`).

## 4. Processes

| Process | Role | Rate |
|---|---|---|
| `can_bridge` | Sends motor commands, reads motor feedback, reports faults | Motor loop (PR-1) |
| `imu_bridge` | Publishes BNO055 orientation, angular velocity, acceleration | 100 Hz |
| `camera_bridge` | Publishes camera frames | 15–30 fps *(est.)* |
| `arm_bridge` | RoArm serial commands, joint feedback, load/contact signal | 10–50 Hz *(est.)* |
| `state_estimator` | Builds the policy's observations: body orientation, velocities, joint states | Policy rate |
| `policy_runner` | Runs the trained PPO policy; outputs leg joint targets | Policy rate (PR-2) |
| `arm_controller` | Arm IK, trajectories, contact-stop behaviour | 50 Hz *(est.)* |
| `supervisor` | Safety: watchdogs, limits, e-stop, mode state machine; final gate on all commands | Motor loop |
| `teleop` | Gamepad/keyboard → velocity commands, modes, arm targets, e-stop | 50 Hz |
| `foxglove_bridge` | Streams selected topics live to Foxglove; records them to MCAP files | 10–30 Hz live |
| `sim_bridge` | Runs MuJoCo and replaces all hardware bridges | Sim rate |

## 5. Performance requirements

In P1 (Python, sim only) these are relaxed: the stack runs in lock-step with the sim and
only needs to keep up with the policy rate. P2/P3 must meet them on the Pi.

- **PR-1** Motor loop (command + feedback for all 12 motors): **≥ 200 Hz** *(est.)*. The RL
  policy runs slower (PR-2), and RobStride motors close their own position/velocity loop in
  impedance mode, so the motor loop does not need 1 kHz.
- **PR-2** Policy rate: **50 Hz**, the rate the current policy was trained at.
- **PR-3** Latency from the latest IMU and joint data to the matching policy action reaching
  the motors: ≤ 5 ms *(est.)*.
- **PR-4** Policy period jitter: ≤ 1 ms at the 99.9th percentile.
- **PR-5** The real-time processes (`can_bridge`, `state_estimator`, `policy_runner`,
  `supervisor`) never allocate memory, block, or log synchronously inside the loop.
- **PR-6** CAN bus load stays ≤ 70 % per bus. *Derived:* 12 motors × (command + reply)
  × ~150 bits ≈ 3,600 bits per cycle, so one 1 Mbit/s bus supports ~190 Hz at 70 % load.
  **PR-1 at 200 Hz therefore needs at least 2 buses.**

## 6. Functional requirements

### 6.1 Motor interface (`can_bridge`)
- **FR-1** Supports the RobStride 00 modes needed: impedance (position + velocity +
  feed-forward torque with Kp/Kd), plus enable, disable and set-zero.
- **FR-2** Publishes each motor's position, velocity, torque, temperature and fault flags.
- **FR-3** Detects a missing motor reply within 3 cycles and reports it to `supervisor`.
- **FR-4** Converts between motor and joint space (direction, zero offset, gear ratio if any),
  using one calibration file per robot.

### 6.2 Walking (`policy_runner`)
- **FR-5** Loads the trained PPO policy from an **ONNX** file, so the same file runs from
  Python (P1) and from Rust (P2). The policy outputs joint targets that are sent to the
  motors in **MIT (impedance) mode** at 50 Hz, as in training.
- **FR-5a** The policy is swappable through config: changing the ONNX file and its
  observation/action definition must not need code changes. This is needed for the later
  policy trained with the arm.
- **FR-6** Builds the observation vector exactly as in training: same order, scaling,
  joint order, default pose and history length. The definition comes from the training
  config and is checked by a test that compares against recorded training data.
- **FR-7** Converts actions to joint targets exactly as in training (action scale, default
  pose offset, Kp/Kd).
- **FR-8** Takes the body velocity command (vx, vy, yaw rate) from `teleop`.
- **FR-9** The sim model matches the one the policy was trained on (masses, joint limits,
  motor gains, joint order); differences are listed and tested **[OPEN: which model was it
  trained on?]**.

### 6.3 Modes (`supervisor`)
- **FR-10** Modes: `passive` (motors off), `damping` (soft hold), `stand` (move to and hold
  the default pose), `sit` (lower to rest), `walk` (policy active).
- **FR-11** A single state machine owns the mode. Transitions are requested via
  request/response and rejected when not allowed (e.g. `walk` only from `stand`,
  `stand` → `walk` only once standing has settled).

### 6.4 Arm and contact sensing
- **FR-12** `arm_controller` accepts joint-space targets and Cartesian end-effector targets.
- **FR-13** `arm_bridge` publishes a contact/pressure signal for the arm: per-joint load from
  the servos and/or a force sensor at the gripper **[OPEN]**.
- **FR-14** The arm can be commanded to press against a surface with a target force and hold
  it (needed for plastering, painting, tiling), and stops automatically when contact force
  exceeds a limit.
- **FR-15** The sim provides the same contact signal (MuJoCo contact forces at the
  gripper / joint torques) so contact behaviour can be developed in P1.
- **FR-16** The current policy was trained **without the arm**. Until a policy trained with
  the arm exists:
  - `walk` is only allowed with the arm in a stowed pose (folded close to the body), and
    `supervisor` rejects arm motion during `walk`.
  - Walking with the arm's mass on the front plate may be degraded in sim; that is accepted
    for P1. It is a known gap, not a bug.
  - The sim can optionally run without the arm (`hardware/dog/dog.xml`) to check the policy
    on the model it was trained for.

### 6.5 Estimation
- **FR-17** `state_estimator` publishes body orientation, angular velocity and projected
  gravity from the BNO055, plus joint states, in the frames the policy expects.

### 6.6 Camera
- **FR-18** `camera_bridge` publishes frames over iceoryx2 for other processes, and a
  compressed stream to Foxglove.
- **FR-19** The sim provides a matching virtual camera at the same mount position.

### 6.7 Teleop
- **FR-20** Gamepad (and keyboard fallback): body velocity, mode changes, arm jog/targets,
  e-stop.

### 6.8 Visualisation, recording and app
- **FR-21** `foxglove_bridge` streams state, commands, IMU, camera, contact forces, modes
  and faults live to Foxglove, including a 3D view of the robot model.
- **FR-22** All topics can be recorded to MCAP files (Foxglove's log format) and replayed
  into the stack to test the estimator and policy offline.
- **FR-23** The TypeScript app reads state and faults through the same bridge; read-only
  at first **[OPEN]**.

## 7. Safety requirements

- **SR-1** Every command stream has a watchdog. If `supervisor` receives no fresh command
  within 3 policy periods, it switches to `damping`, then to `passive` after 2 s *(est.)*.
- **SR-2** `supervisor` is the only process whose commands `can_bridge` and `arm_bridge`
  accept. Controllers publish proposed commands; `supervisor` checks and forwards them.
- **SR-3** `supervisor` clamps commands to the position, velocity and torque limits per joint.
- **SR-4** An e-stop (software and hardware) cuts motor torque in every mode; recovery
  requires an explicit operator action.
- **SR-5** Motor over-temperature, a fault flag, or a lost motor reply switches the robot
  to `damping`.
- **SR-6** If `can_bridge` loses contact with `supervisor`, it disables the motors itself.
- **SR-7** Body tilt beyond a threshold (e.g. 45°) switches the robot to `damping`.
- **SR-8** Arm contact force above a hard limit stops the arm immediately, in any mode.

## 8. Simulation, development and test

- **SIM-1** `sim_bridge` publishes the same services as `can_bridge`, `imu_bridge`,
  `arm_bridge` and `camera_bridge`, including the RobStride impedance behaviour and the
  BNO055 noise and rate.
- **SIM-2** Runs in lock-step (deterministic, for tests) or real-time (for teleop).
- **SIM-3** Uses the models in `hardware/` without hand edits.
- **SIM-4** Runs headless in CI.
- **DT-1** Python in P1; Rust for the real-time processes from P2. `sim_bridge` and
  `foxglove_bridge` may stay in Python.
- **DT-2** Every process can be started on its own and is configured by a TOML file.
- **DT-3** One launch command starts the whole stack, for sim or for hardware.
- **DT-4** CI runs unit tests, plus an integration test against the headless sim that
  checks stand → walk forward 1 m → stand → sit.

## 9. Development and deployment model

### Now: native development
- **DEP-1** Development is **native**: every process runs directly on the host OS (the
  development machine in P1, the Raspberry Pi in P3), started from the source tree,
  without containers.
- **DEP-2** The native setup is reproducible: one setup script/lockfile installs the
  dependencies (Python environment, iceoryx2, MuJoCo, ONNX Runtime), and one launch
  command starts the stack (DT-3).
- **DEP-3** Nothing in the code may assume it runs natively: no hard-coded absolute paths,
  user names or host names. Paths, device names (`can0`, serial ports, camera) and
  iceoryx2 settings come from config or environment variables, so the same processes run
  later inside containers unchanged.

### Later: fleet management with balenaCloud
- **DEP-4** The robots are later managed as a fleet with **balenaCloud**: balenaOS on each
  Pi, the stack deployed as containers, over-the-air updates, and per-device configuration
  through balena environment variables.
- **DEP-5** The process boundaries (§4) map directly to containers, or to a few containers
  grouping processes, without code changes.
- **DEP-6** All containers that talk over iceoryx2 must share its shared memory and its
  service/discovery files (shared IPC namespace, `/dev/shm`, and the iceoryx2 directories).
  This must be shown to work on balenaOS before P3 (risk R-5).
- **DEP-7** Hardware access from containers: CAN interfaces (SocketCAN), the BNO055
  serial/I²C device, the arm's USB serial port, and the camera.
- **DEP-8** Per-robot data (motor calibration, zero offsets, IMU calibration, robot ID)
  lives outside the containers (persistent volume or device variables), so an update
  never overwrites calibration.
- **DEP-9** Recordings (MCAP) are stored on a persistent volume and can be uploaded off
  the robot.
- **DEP-10** Updates must be safe: an update is never applied while the robot is in a
  powered mode (`stand`/`walk`), and the robot comes back in `passive` after a restart.

## 10. Risks

- **R-1 Raspberry Pi real-time performance.** PR-1 to PR-5 on a Pi need a real-time
  kernel, isolated CPU cores, and Rust in the real-time path. The camera stream and Foxglove
  bridge must not starve the control loop.
- **R-2 CAN on the Pi.** The Pi has no built-in CAN. SPI HATs based on the MCP2515 chip
  struggle at high frame rates on a 1 Mbit/s bus. A USB-CAN adapter, or a HAT with a
  faster controller, is likely needed, with 2 buses for PR-6.
- **R-3 BNO055 on the Pi.** The BNO055 uses I²C clock stretching, which the Pi's I²C
  controller handles poorly. UART mode is the usual workaround. Its fused output tops out
  at 100 Hz, which is enough for a 50 Hz policy.
- **R-4 Sim-to-real gap for the policy.** The policy only transfers if the sim matches the
  training setup (FR-6, FR-9) and the real robot (masses, motor gains, latency).
- **R-5 iceoryx2 across balena containers.** Zero-copy needs the containers to share
  memory and iceoryx2's files. This is not tested yet on balenaOS; prove it with a
  two-container publisher/subscriber test early, before relying on it.
- **R-6 Real-time on balenaOS.** balenaOS ships its own kernel, so a real-time kernel and
  CPU isolation (R-1) may not be available or easy to configure. Real-time scheduling
  inside containers also needs extra privileges. Check what balenaOS supports on the
  chosen Pi before P3; this could decide whether the real-time path needs another approach.

## 11. Open questions

1. **Policy observation/action definition:** the ONNX file shows the input and output
   sizes, but not their meaning (order, scaling, default pose, Kp/Kd, history length).
   These must come from the training code/config. Where is that, and in which simulator was
   it trained?
2. **CAN:** which adapter, and how many buses?
3. **Pi:** which model (Pi 5 assumed)? Real-time kernel?
4. **BNO055:** wired over I²C or UART?
5. **Arm contact sensing:** servo load feedback only, or add a force sensor at the gripper?
   Which tasks need what force accuracy?
6. **Camera:** model, resolution, frame rate, and what it is used for (operator view,
   perception later)?
7. **Arm link:** USB serial or Wi-Fi to the ESP32?
8. **Message schema:** custom TOML plus a generator, or Python definitions that generate Rust?
9. **App:** read-only at first, or can it send commands?
