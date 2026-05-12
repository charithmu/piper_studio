# Piper Studio Multi-Agent Orchestration

This file is the supervisor-authored deployment plan for parallel agent
development of the Piper Studio perception and manipulation stack. It is the
single source of truth for which work runs in parallel, which work is gated,
and what each agent is responsible for.

Read order for any agent starting work:

1. `AGENTS.md` (workspace)
2. `DESIGN.md` — especially **Pinned Cross-Package Contract**
3. The package-local `AGENTS.md` and `README.md`
4. `ORCHESTRATION.md` (this file) — agent brief and acceptance gates

## Pinned Decisions (do not redebate)

- **Perception scope (first milestone): AprilTag only.** `agx_arm_detect` and
  `agx_arm_graspgen` are AprilTag-driven for now. Future detection modes
  attach additively under `/detections/<mode>` without reshaping contracts.
- **Detection messages:** `apriltag_msgs/msg/AprilTagDetectionArray` on
  `/detections/apriltag`, `geometry_msgs/msg/PoseStamped` on
  `/detections/tag_pose`, plus TF frames prefixed `tag`.
- **Grasp messages:** `geometry_msgs/msg/PoseArray` on `/grasp/candidates`,
  `visualization_msgs/msg/MarkerArray` on `/grasp/debug_markers`.
- **Camera mount parent frame:** `gripper_base` (real URDF link from the
  standard AgileX gripper xacros). `tcp_link` is MoveIt-only.
- **Sequencing:** `agx_arm_motion` backend generalization happens before
  `agx_arm_manipulation` implementation starts.

These are reflected in `contracts.py` in each package — agents should import
from there rather than hardcoding strings.

## Dependency Graph

Ownership legend:

- **Upstream-frozen** (do not edit unless coordinated with upstream):
  `pyAgxArm`, `agx_arm_ros` (and its subpackages `agx_arm_ctrl`,
  `agx_arm_description`, `agx_arm_moveit`, `agx_arm_msgs`).
- **Workspace-owned** (we own them; agents may modify when needed to keep
  sim/real parity or to unblock their feature): `agx_arm_gzsim`,
  `agx_arm_motion`, `agx_arm_eyes`, `agx_arm_detect`, `agx_arm_graspgen`,
  `agx_arm_manipulation`.

```
                       pyAgxArm  (upstream-frozen, SDK)
                            │
                     agx_arm_ros (upstream-frozen, ROS bridge + URDF + MoveIt config)
                            │
        ┌───────────────────┼─────────────────────────────┐
        │                   │                             │
  agx_arm_gzsim       agx_arm_motion-gen           agx_arm_eyes-core
  (workspace-owned;   (Wave 1, Agent M)            (Wave 1, Agent E1)
  improvable for
  sim parity)
                            │                             │
                            │                             │ (interface stable)
                            │                             ▼
                            │                       ┌─────┴─────────┐
                            │                       │               │
                            │                 agx_arm_eyes    agx_arm_detect
                            │                 vendors+cal     (Wave 2, Agent D)
                            │                 (Wave 2,        AprilTag wiring
                            │                 E2/E3/E-cal)         │
                            │                                      ▼
                            │                            agx_arm_graspgen
                            │                            (Wave 2, Agent G)
                            │                                      │
                            └──────────────┬───────────────────────┘
                                           ▼
                                  agx_arm_manipulation
                                  (Wave 3, Agent Man)
```

Hard dependencies (must finish before consumer starts):

- `agx_arm_motion-gen` (M) ───▶ `agx_arm_manipulation` (Man)
- `agx_arm_eyes-core` (E1) ───▶ `agx_arm_detect` (D),
  `agx_arm_eyes-vendors` (E2/E3), `agx_arm_eyes-calibration` (E-cal)
- `agx_arm_detect` (D) ───▶ `agx_arm_graspgen` (G)
- `agx_arm_graspgen` (G) ───▶ `agx_arm_manipulation` (Man)

Soft dependencies (nice-to-have but mockable):

- D can start against a fake AprilTag publisher and a recorded bag while E1
  finishes, provided D only depends on contracts in
  `agx_arm_detect/contracts.py`.
- G can start against a fake `/detections/apriltag` publisher while D is in
  flight, provided G only depends on contracts.

## Human Checkpoint Discipline

Hard rule: **no agent in wave N+1 is dispatched until the human has signed
off on the checkpoint at the end of wave N.** Same rule for mid-wave
checkpoints inside Wave 1. The supervisor's job is to keep checkpoints
small and frequent so that manual review never accumulates into a
multi-hour audit.

Each checkpoint is bounded:

- **Time budget per checkpoint: ≤ 30 minutes of human attention.** If a
  checkpoint can't be covered in that window, split it.
- The human (eranga) runs the listed commands locally, watches the
  listed signals, and answers a yes/no sign-off question. The supervisor
  records the result in `log/checkpoints.md` (create if missing) with
  date, wave, pass/fail, and any defects to feed back to the agent.
- If the checkpoint fails, the supervisor sends the failure summary back
  to the responsible agent for a fix-pass before any downstream agent
  starts. Do not paper over a failed checkpoint by moving forward.
- No new agent dispatch between sign-off and the next wave's start
  without re-reading this file — context drift is the failure mode this
  whole discipline exists to prevent.

Each "Human Checkpoint" block below has the same shape:

1. **What to install / build / source** (one-line commands).
2. **What to launch** (one terminal per line).
3. **What to look for** (the explicit visual or log evidence).
4. **Sign-off question** (single yes/no).
5. **Defects to feed back** (free-form notes section).

## Wave Plan

### Wave 0 — Foundation (sequential, single agent)

Goal: unblock everything else by pinning the motion backend abstraction and
the camera launch surface.

| Agent | Package | Acceptance |
|---|---|---|
| **M** | `agx_arm_motion` generalization | Same `move_to_pose` and `pose_goal_server` work against both `--backend sim` and `--backend real` paths; no `agx_arm_gzsim` hardcoding; `use_sim_time` driven by argument; passes a smoke launch in sim. |
| **E1** | `agx_arm_eyes` core launch surface + one camera (RealSense D435) | `ros2 launch agx_arm_eyes bringup.launch.py camera_model:=realsense_d435 mount_name:=<default>` brings up all `/wrist_camera/...` topics including `color/image_rect` and the `gripper_base -> camera_mount_link -> camera_link -> optical` TF chain. |

M and E1 are independent and **can run in parallel** (no shared files, no
shared contracts). Both must complete before Wave 1 closes.

#### → Human Checkpoint 0 (after Wave 0)

Budget: ~20 minutes. Goal: confirm the motion backend abstraction and the
camera launch surface each work in isolation in sim before any perception
agent is dispatched.

1. Build and source:
   ```bash
   colcon build --packages-select agx_arm_motion agx_arm_eyes agx_arm_gzsim
   source install/setup.bash
   ```
2. Launch (one terminal each):
   - T1: `./scripts/piper_studio.sh moveit --backend sim`
   - T2: `./scripts/piper_studio.sh motion-once --x 0.30 --y 0.0 --z 0.25 --pitch 1.5708`
   - T3 (separate run, after T1/T2): `ros2 launch agx_arm_eyes bringup.launch.py camera_model:=realsense_d435`
3. Look for:
   - T1: `move_group: You can start planning now!`, no `agx_arm_gzsim` import errors from `agx_arm_motion`.
   - T2: Arm visibly reaches the target pose in RViz/Gazebo; node exits cleanly.
   - T3: `ros2 topic list` shows `/wrist_camera/color/image_raw`, `/wrist_camera/color/image_rect`, `/wrist_camera/color/camera_info`, `/wrist_camera/depth/...`, `/wrist_camera/points`. `ros2 run tf2_ros tf2_echo gripper_base camera_link` returns a static transform.
4. Sign-off question: *Did motion run against the sim move_group with the new backend arg, AND did the camera launch publish the standard topic + TF set?*
5. Defects fed back to: Agent M and/or Agent E1.

### Wave 1 — Parallel perception fan-out

Run all of these concurrently once Wave 0 is done.

| Agent | Package | Depends on | Acceptance |
|---|---|---|---|
| **E2** | `agx_arm_eyes` Orbbec Astra U3 adapter | E1 | One-command launch publishes the same standard topics for Astra U3. |
| **E3** | `agx_arm_eyes` OAK-D Max adapter | E1 | Same standard topics for OAK-D Max. |
| **E-cal** | `agx_arm_eyes` eye-in-hand calibration | E1 | MoveIt calibration launch surface; profiles stored under `config/calibration/<arm>/<camera>/<mount>/<profile>.yaml`; explicit launch-time selection. |
| **D** | `agx_arm_detect` AprilTag pipeline | E1 (`image_rect` + `camera_info` topic contract) | Subscribes to `agx_arm_detect.contracts.DEFAULT_INPUT_TOPICS`; publishes `apriltag_msgs/AprilTagDetectionArray` on `/detections/apriltag`, `geometry_msgs/PoseStamped` on `/detections/tag_pose`, and TF `tag_<id>` frames. Validates against a bag or a sim wrist-camera. |
| **G** | `agx_arm_graspgen` AprilTag heuristics | D's message contract (can start against a fake publisher) | Generates `geometry_msgs/PoseArray` of approach / grasp / retreat candidates from a tag pose, in `base_link`. Visualizable via `MarkerArray`. |

Why these are safe in parallel:

- E2 and E3 touch only their vendor launch files and a vendor config block;
  no shared code with each other or with D/G.
- E-cal touches `launch/` and `config/calibration/` only.
- D and G touch separate packages; their interface is already pinned in
  `contracts.py`. Both must import contract constants rather than hardcoding.

Avoid: any edit to `agx_arm_eyes/contracts.py` or
`agx_arm_detect/contracts.py` during Wave 1 — the contract is frozen for the
wave.

**Wave 1 is split into two human checkpoints** so the human is not asked to
review five concurrent agents at once. Dispatch order:

- Dispatch group **1a**: E2, E3, E-cal in parallel. Sign off at Checkpoint 1a.
- Dispatch group **1b**: D and G in parallel. Sign off at Checkpoint 1b.

#### → Human Checkpoint 1a (after E2 + E3 + E-cal)

Budget: ~25 minutes. Goal: confirm every vendor camera bringup conforms to
the same contract E1 set, and that the calibration save path is real.

1. Build and source:
   ```bash
   colcon build --packages-select agx_arm_eyes
   source install/setup.bash
   ```
2. For each of the three camera models, in a fresh terminal:
   ```bash
   ros2 launch agx_arm_eyes bringup.launch.py camera_model:=realsense_d435
   # then in another terminal: ros2 topic list | grep wrist_camera
   ```
   Repeat with `camera_model:=astra_u3` and `camera_model:=oakd_max`.
   (If a physical camera is unavailable, ask the agent to confirm the
   launch resolves and parameters are wired; mark that vendor as
   "launch-only validated" in the defects section.)
3. Calibration: run the eye-in-hand calibration launch with
   `calibration_profile:=test_<date>` against any available camera; confirm a
   YAML lands under `src/agx_arm_eyes/config/calibration/<arm>/<camera>/<mount>/test_<date>.yaml`
   with the metadata fields (arm, effector, camera, mount, date, operator).
4. Look for:
   - Topics for each vendor are identical to E1's list (same names under
     `/wrist_camera/...`), even if data only flows when the device is
     present.
   - TF chain `gripper_base -> camera_mount_link -> camera_link` resolves
     in all three cases.
   - Calibration YAML is human-readable and stored in-repo, not in `$HOME`.
5. Sign-off question: *Do all three camera launches expose the same
   contract, and does the calibration profile land under the package
   config tree?*
6. Defects fed back to: E2, E3, or E-cal individually.

#### → Human Checkpoint 1b (after D + G)

Budget: ~25 minutes. Goal: confirm the AprilTag detection→grasp candidate
path is correct on a known input before manipulation is dispatched.

1. Build and source:
   ```bash
   colcon build --packages-select agx_arm_detect agx_arm_graspgen
   source install/setup.bash
   ```
2. Launch (in order, one terminal each):
   - T1: `ros2 launch agx_arm_eyes bringup.launch.py camera_model:=realsense_d435` (or replay a bag with the same topics).
   - T2: `ros2 launch agx_arm_detect detect.launch.py`
   - T3: `ros2 launch agx_arm_graspgen graspgen.launch.py`
   - T4: `ros2 run rviz2 rviz2` with the `/grasp/debug_markers` and `tag_*` TF frames added.
3. Place a known AprilTag in front of the camera (or play the bag).
4. Look for:
   - `ros2 topic echo /detections/apriltag --once` shows
     `apriltag_msgs/msg/AprilTagDetectionArray` with the right tag id.
   - `ros2 topic echo /detections/tag_pose --once` shows a
     `geometry_msgs/msg/PoseStamped` in a sensible frame.
   - `ros2 topic echo /grasp/candidates --once` returns a non-empty
     `PoseArray` whose poses make geometric sense around the tag.
   - RViz shows the `tag_<id>` TF and grasp markers visibly hovering on
     the correct face of the tag (top-down for a flat tag).
5. Sign-off question: *Does the detection round-trip publish the pinned
   message types, and do the grasp candidates look physically plausible
   in RViz?*
6. Defects fed back to: Agent D or Agent G.

### Wave 2 — Manipulation (sequential, single agent)

| Agent | Package | Depends on | Acceptance |
|---|---|---|---|
| **Man** | `agx_arm_manipulation` | M + D + G | `move_to_detected_tag_pose` works in sim end-to-end. A basic pick-and-place demo runs in sim using `agx_arm_motion` for primitives and `agx_arm_graspgen` for candidates. No MoveIt glue duplicated. |

#### → Human Checkpoint 2 (after Man)

Budget: ~30 minutes. Goal: confirm the sim pick-and-place demo runs end to
end and uses the lower layers as designed.

1. Build and source:
   ```bash
   colcon build --packages-select agx_arm_manipulation
   source install/setup.bash
   ```
2. Launch (one terminal each):
   - T1: `./scripts/piper_studio.sh moveit --backend sim`
   - T2: `ros2 launch agx_arm_eyes bringup.launch.py camera_model:=realsense_d435` (or sim wrist camera fixture from `agx_arm_gzsim`).
   - T3: `ros2 launch agx_arm_detect detect.launch.py`
   - T4: `ros2 launch agx_arm_graspgen graspgen.launch.py`
   - T5: `ros2 launch agx_arm_manipulation pick_place_demo.launch.py`
3. Place a tagged object in the sim scene.
4. Look for:
   - `move_to_detected_tag_pose` reaches the tag in RViz/Gazebo.
   - `pick` closes the gripper, `place` opens it at the target.
   - `/manipulation/status` topic prints intelligible state transitions.
   - `agx_arm_manipulation` does not import MoveIt directly; calls flow
     through `agx_arm_motion`. (Spot-check by `grep -R "moveit" src/agx_arm_manipulation/`.)
5. Sign-off question: *Did a complete pick-and-place run in sim using the
   composed stack, with status topic narrating the sequence?*
6. Defects fed back to: Agent Man (or upstream wave if a regression
   surfaces in motion/detect/graspgen).

### Wave 3 — Hardware port (sequential, supervisor-gated)

Port the Wave 2 demo to real hardware. Same task-level APIs, swap backend
and camera. Not a parallel deploy.

#### → Human Checkpoint 3 (after hardware port)

Budget: ~30 minutes per camera × hardware run. Goal: confirm the same
task-level interface from Checkpoint 2 runs on real hardware without code
forks.

1. Activate CAN, source:
   ```bash
   ./scripts/piper_studio.sh setup-can --can-name can0 --bitrate 1000000
   source install/setup.bash
   ```
2. Launch (one terminal each):
   - T1: `./scripts/piper_studio.sh moveit --backend real --can-port can0 --arm-type piper --effector-type agx_gripper`
   - T2: `ros2 launch agx_arm_eyes bringup.launch.py camera_model:=<real_camera> calibration_profile:=<latest>`
   - T3: `ros2 launch agx_arm_detect detect.launch.py`
   - T4: `ros2 launch agx_arm_graspgen graspgen.launch.py`
   - T5: `ros2 launch agx_arm_manipulation pick_place_demo.launch.py`
3. Stand by the E-stop. Place a tagged object in the workcell.
4. Look for: parity with the sim run — same topics, same status sequence,
   no new MoveIt or vendor imports in the high-level packages.
5. Sign-off question: *Did the same demo run on real hardware with no
   code changes outside calibration profile and backend args?*
6. Defects fed back to: whichever layer regressed.

## Parallel vs Sequential Summary

| Pair | Run as | Reason |
|---|---|---|
| M ‖ E1 | Parallel | Disjoint packages, disjoint contracts. |
| E1 → E2, E3, E-cal | Sequential start, then parallel | All three need E1's stable launch surface, then they touch disjoint vendor / calibration paths. |
| E1 → D | Sequential start | D needs the rectified-image topic contract. Once stable, D runs in parallel with E2/E3/E-cal. |
| D ‖ G | Parallel (soft) | G can develop against a fake `/detections/apriltag` publisher using pinned contracts. Final integration after D lands. |
| M, G → Man | Sequential | Manipulation composes motion + grasp candidates. |

## Agent Briefs (copy into deployments)

Each brief is self-contained: the agent will not see this conversation.
Each brief assumes the workspace at
`/home/atlasdev/projects/ros2_projects/piper_studio`.

### Agent M — `agx_arm_motion` backend generalization

*Model: **Sonnet** (`claude-sonnet-4-6`). Tech-lead role.*

> You are implementing the backend abstraction in `src/agx_arm_motion`.
> Read `AGENTS.md`, `DESIGN.md` (Pinned Cross-Package Contract), and
> `src/agx_arm_motion/README.md` first. The current launch files hardcode
> `agx_arm_gzsim` robot-description and controller assets and force
> `use_sim_time:=true`. Refactor so that the same `move_to_pose` and
> `pose_goal_server` entrypoints accept a `backend:=sim|real` argument and
> a shared MoveIt config builder selects assets accordingly. Preserve the
> existing public launch argument surface. Validate with
> `./scripts/piper_studio.sh moveit --backend sim` and confirm the new
> motion launch works against the sim move_group without any
> `agx_arm_gzsim` references inside `agx_arm_motion` itself.
> `agx_arm_gzsim` is workspace-owned, so you MAY edit it if it is the
> right home for sim-side launch glue (e.g. a sim MoveIt bringup
> entrypoint that `agx_arm_motion` calls). Keep `agx_arm_ros` and
> `pyAgxArm` untouched — those are upstream-frozen.

### Agent E1 — `agx_arm_eyes` core launch surface (RealSense)

*Model: **Sonnet** (`claude-sonnet-4-6`). Tech-lead role.*

> You are implementing the core camera adapter in `src/agx_arm_eyes`.
> Read `AGENTS.md`, `DESIGN.md` (Pinned Cross-Package Contract),
> `src/agx_arm_eyes/AGENTS.md`, `src/agx_arm_eyes/README.md`, and
> `src/agx_arm_eyes/agx_arm_eyes/contracts.py`. Implement a single
> `launch/bringup.launch.py` that wraps the upstream `realsense2_camera`
> launch and remaps onto the namespace and topics defined in `contracts.py`,
> including `/wrist_camera/color/image_rect` (use `image_proc` if needed).
> Publish the static TF chain
> `gripper_base -> camera_mount_link -> camera_link -> <optical frames>`
> from a config under `config/mounts/`. Add launch args `camera_model`,
> `mount_name`, `namespace`, `parent_frame`, `calibration_profile`,
> `align_depth`, `publish_pointcloud`. Only RealSense in this brief; Astra
> and OAK-D are separate agents. Do not edit other packages.

### Agent E2 — `agx_arm_eyes` Astra U3 adapter

*Model: **Haiku** (`claude-haiku-4-5-20251001`). Developer role. If you hit a contract ambiguity, stop and escalate to the supervisor; do not redesign the contract.*

> Add Orbbec Astra U3 support to `src/agx_arm_eyes`. Wave 1; E1 has
> already landed the core launch surface and the contract is frozen. Wrap
> the upstream `orbbec_camera` launch and remap onto the standard
> `/wrist_camera/...` topics defined in
> `src/agx_arm_eyes/agx_arm_eyes/contracts.py`. Do not change the contract
> or the TF chain. Add `launch/orbbec.launch.py` and any vendor-specific
> config under `config/vendors/orbbec/`. Validate by launching with
> `camera_model:=astra_u3`.

### Agent E3 — `agx_arm_eyes` OAK-D Max adapter

*Model: **Haiku** (`claude-haiku-4-5-20251001`). Developer role. If you hit a contract ambiguity, stop and escalate to the supervisor; do not redesign the contract.*

> Add OAK-D Max support to `src/agx_arm_eyes`. Wave 1; E1 has landed the
> core launch surface. Wrap `depthai_ros_driver` and remap onto the
> standard topics. Add `launch/oakd.launch.py` and vendor config under
> `config/vendors/oakd/`. Validate by launching with
> `camera_model:=oakd_max`.

### Agent E-cal — Eye-in-hand calibration

*Model: **Haiku** (`claude-haiku-4-5-20251001`). Developer role. If the storage convention or profile-metadata fields look under-specified, stop and escalate.*

> Add MoveIt eye-in-hand calibration support to `src/agx_arm_eyes`. Wave 1;
> E1 has landed. Add a launch file that wires the MoveIt calibration
> tooling against the selected wrist camera and a parameter-driven save
> path under `config/calibration/<arm>/<camera>/<mount>/<profile>.yaml`.
> Persist profile metadata (arm type, effector type, camera model, mount,
> date, operator notes). Add a launch-time selector for the active profile
> and exercise it from `bringup.launch.py`. Do not change the topic or TF
> contract.

### Agent D — `agx_arm_detect` AprilTag pipeline

*Model: **Sonnet** (`claude-sonnet-4-6`). Tech-lead role.*

> Implement the AprilTag detection node in `src/agx_arm_detect`. Read
> `AGENTS.md`, `DESIGN.md` (Pinned Cross-Package Contract),
> `src/agx_arm_detect/AGENTS.md`, `src/agx_arm_detect/README.md`, and
> `src/agx_arm_detect/agx_arm_detect/contracts.py`. Subscribe to the topics
> in `DEFAULT_INPUT_TOPICS`. Publish
> `apriltag_msgs/msg/AprilTagDetectionArray` on `/detections/apriltag`,
> `geometry_msgs/msg/PoseStamped` on `/detections/tag_pose`, and broadcast
> TF as `<DEFAULT_TAG_FRAME_PREFIX>_<id>`. Scope is AprilTag only. Wrap or
> reuse an existing ROS 2 AprilTag detector — do not write a detector from
> scratch. Validate against a bag or the sim wrist-camera. Do not depend
> on RealSense / Orbbec / OAK-D specific topics; only the standard
> `/wrist_camera/...` interface.

### Agent G — `agx_arm_graspgen` AprilTag heuristics

*Model: **Haiku** (`claude-haiku-4-5-20251001`). Developer role. The contracts and approach templates are pinned; if you find yourself wanting to extend the message contract, stop and escalate to the supervisor.*

> Implement the heuristic grasp generator in `src/agx_arm_graspgen`. Read
> `AGENTS.md`, `DESIGN.md` (Pinned Cross-Package Contract),
> `src/agx_arm_graspgen/AGENTS.md`, `src/agx_arm_graspgen/README.md`, and
> `src/agx_arm_graspgen/agx_arm_graspgen/contracts.py`. Subscribe to the
> topics in `DEFAULT_INPUT_TOPICS`. From a tag pose, emit approach,
> grasp, and retreat candidates as `geometry_msgs/msg/PoseArray` on
> `/grasp/candidates` in `base_link`. Emit a parallel
> `visualization_msgs/msg/MarkerArray` on `/grasp/debug_markers`. Start
> with deterministic, parameter-driven top-down and tag-aligned templates.
> Do not execute motion. If Agent D is not yet done, validate against a
> fake `/detections/apriltag` publisher using the pinned contract.

### Agent Man — `agx_arm_manipulation`

*Model: **Sonnet** (`claude-sonnet-4-6`). Tech-lead role.*

> Implement task-level behaviors in `src/agx_arm_manipulation`. Wave 2;
> Agents M, D, and G have already landed. Read `AGENTS.md`, `DESIGN.md`
> (Pinned Cross-Package Contract), `src/agx_arm_manipulation/AGENTS.md`,
> `src/agx_arm_manipulation/README.md`, and
> `src/agx_arm_manipulation/agx_arm_manipulation/contracts.py`. Use
> `agx_arm_motion` for motion primitives — do not duplicate MoveIt setup
> or execution. Implement `move_to_detected_tag_pose`, `pick`, `place`,
> and a `pick_place_demo` that runs in sim end-to-end. Surface task-level
> status on `DEFAULT_STATUS_TOPIC`.

## Supervisor Verification After Each Wave

The supervisor runs an automated pre-check, then hands off to the human
for the wave's checkpoint above. Supervisor pre-check:

1. Confirm contracts in each package's `contracts.py` were not silently
   changed.
2. Confirm no agent edited another package's source, and that
   `agx_arm_ros` and `pyAgxArm` were not touched.
3. Run the package builds named in the checkpoint and report any compile
   failure back to the responsible agent before the human is involved.
4. Prepare the checkpoint command block above so the human can copy-paste.
5. After the human signs off, append a one-line entry to
   `log/checkpoints.md` (date, wave, pass/fail, defects). Update
   `TODO.md` checkboxes only on pass.

Do not dispatch the next wave until the human checkpoint passes.

## Model Assignment Policy

Locked per role. Pass these via the `Agent` tool's `model` parameter when
dispatching.

| Role | Model | Model ID | Rationale |
|---|---|---|---|
| Supervisor / designer (this agent) | Opus | `claude-opus-4-7` | Owns architecture, contracts, checkpoints. |
| Tech-lead agents | Sonnet | `claude-sonnet-4-6` | Set or extend a contract; orchestrate lower layers; refactor across packages. |
| Developer agents | Haiku | `claude-haiku-4-5-20251001` | Implement against a pinned, narrow contract. |

Per-agent assignment:

| Agent | Role | Model |
|---|---|---|
| **M** — `agx_arm_motion` generalization | Tech lead (sets the sim/real backend abstraction; may extend `agx_arm_gzsim`) | Sonnet |
| **E1** — `agx_arm_eyes` core launch surface | Tech lead (defines the camera contract every vendor must match) | Sonnet |
| **D** — `agx_arm_detect` AprilTag pipeline | Tech lead (defines the detection wrapping + frame conventions consumed by G and Man) | Sonnet |
| **Man** — `agx_arm_manipulation` | Tech lead (composes motion + detect + graspgen) | Sonnet |
| **E2** — Astra U3 adapter | Developer (vendor wrapper against E1's frozen contract) | Haiku |
| **E3** — OAK-D Max adapter | Developer (vendor wrapper against E1's frozen contract) | Haiku |
| **E-cal** — eye-in-hand calibration | Developer (wires MoveIt calibration tooling against E1's launch) | Haiku |
| **G** — `agx_arm_graspgen` heuristics | Developer (deterministic geometry against D's pinned contract) | Haiku |

If a Haiku developer hits a contract ambiguity, it stops and escalates to
the supervisor rather than improvising — the cheaper model is not the
place to make architecture calls.

## Tools the Supervisor Will Use to Deploy Agents

- `Agent` with `subagent_type: general-purpose` and the briefs above for
  each parallel worker. **Always pass `model: haiku` or `model: sonnet`
  per the table above.** The supervisor itself remains on Opus.
- Independent agents in the same wave should be launched in a single
  message (multiple `Agent` tool calls) so they run concurrently.
- `isolation: worktree` for each agent so concurrent edits do not collide
  on disk. The supervisor merges worktrees back after acceptance.
- `Plan` agent (Sonnet by default is fine) if a wave-level design call
  needs to be reconsidered.
- `Bash` for the smoke-test commands above.
- Avoid `Explore` for execution work; use it only when an agent needs
  read-only context that exceeds its own search budget.
