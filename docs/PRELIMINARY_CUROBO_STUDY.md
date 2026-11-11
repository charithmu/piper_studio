# Preliminary study: cuRobo (v2) with the Piper

Written 2026-11-08 by a reading-and-checking pass, **before any test was run**. Nothing here was executed on the GPU. It is the starting point for the study at `~/projects/ml_tests/experiments/curobo_piper_study` (cuRobo clone and papers are there). Where something is my reading rather than a verified fact it is marked *(verify)*. Treat the whole document as hypotheses to check, and correct it as you learn.

> **Update (2026-11-08, later):** the arm is on its latest firmware and calibrated; firmware values now win over the vendor URDF (see `DECISIONS.md`, `CLEANUP.md` G). Resolved since this was written:
> joint 6 is ±170° (firmware and vendor URDF agree), velocity 3.0 rad/s and acceleration 5.0 rad/s² are firmware values and in the description, the gripper stroke is 0.07 m, per-link masses
> are in `docs/description/MASSES.md`. Still open: torque limits, jerk, measured masses, camera and stand. Read the numbers below with that in mind.
>
> **Update 3 (2026-11-08, end of day):** the pose rules changed (`docs/real_arm/arms/piper-01.md`, "Poses and the start / stop sequence"): `rest` is now the internal `park`; the powered-off arm is the *slumped* pose, never a target; downstream code knows only `zero` (joints 2/3 at +0.01 / -0.01 rad) and calls `Piper.start()` / `stop()`. The guard is strict (nothing outside the limits, nothing from the slumped pose). The wrist camera stand is on `gripper_base` (4.5 mm decided) with a calibration override `camera_calibration:=`. For planners: `export_urdf.py --gripper-width W` gives a flat URDF with the gripper fixed, and `piper_core.RealArm` (written, tested only against a fake SDK) is the planner-facing real-arm interface (`start`, `execute(trajectory)`, `stop`). The tables below are the original 08:00 reading, kept for the record.

## 1. Decision and organisation

- cuRobo is a **test and planning environment**, not a dependency of Piper Studio. Piper Studio keeps offering ROS-based and direct interfaces to downstream users.
- Do the work in a **separate study repo** (`ml_tests/experiments/curobo_piper_study`), not here: cuRobo brings a heavy GPU stack (torch, CUDA wheels, Warp) that must not mix with this repo's ROS-ABI-pinned environment (NumPy 1.26). The study takes Piper Studio's description as input and talks to the ROS side over topics or files.
- Change Piper Studio only where the study proves the contract lacks something, and in planner-neutral terms (section 5). If a generated cuRobo model (spheres, config) proves stable it could later return here as an optional, checked artifact, like the generated macro. Not before.
- Principle (user, 2026-11-08): **cuRobo does the physical checks and the optimisation. We give it a correct description and do not re-implement the same checks downstream.**

## 2. What cuRobo is (from its docs, code and the three papers)

- Version: latest tag `v0.9.0`; the study clones `main` at `5e4f23d` (2026-11-06). Public API: `curobo.motion_planner`, `inverse_kinematics`, `kinematics`, `scene`, `types`, `perception`, `servo`. Units: metres, radians, seconds, `wxyz` quaternions, float32 CUDA tensors.
- **Control level: joint position.** Output is a timed joint trajectory (position, velocity, acceleration). `JointTargetServo` turns a target plus the measured state into limited fixed-rate position commands (its `kp`/`kd` are internal smoothing, not torque). No example sends torque.
- **Torque is a planning constraint, not an output.** cuRoboV2 / V2-Fast plan *torque-limited, payload-aware* motion with differentiable inverse dynamics (RNEA); payload mass/COM/inertia can be updated at run time. V2-Fast adds TimeOpt (traversal speed under velocity, acceleration, jerk and torque limits). The V2-Fast paper reports that executed velocities can exceed limits even when plans are velocity-feasible, and treats torque as essential.
- **Real-robot precedent:** V2-Fast was deployed on the I2RT YAM, a small low-cost arm, with a stereo camera, SceneFields (depth, detections, robot masks), GraspGenX and cuRobo on one GPU (23 ms grasp motion planning, 5.9 s pick-and-drop cycles). The closest known precedent for Piper. How the YAM limits and tracking were obtained is the first thing to read.
- **Robot model:** a URDF plus mesh assets. `RobotBuilder` fits collision spheres (MorphIt) and a self-collision ignore matrix and writes YAML or XRDF. The Franka config shows the structure: `lock_joints` for fingers, `tool_frames`, `cspace` (default pose, `max_acceleration`, `max_jerk`, scales), `self_collision_buffer`, `self_collision_ignore`, `extra_links` (attached object). Mimic joints are understood by the parser *(verify)*.
- **Perception:** `Mapper` integrates `CameraObservation` (RGB, depth, camera-to-world pose, intrinsics) into a TSDF and ESDF. `RobotSegmenter` removes the arm's own pixels from depth using the collision spheres and the joint state. The live example (RealSense + Franka MPC) uses a **fixed** camera with a hand-set pose; there is no wrist camera or hand-eye support. The example applies its own axis-convention rotation to the camera pose, so the optical-frame convention must be tested.
- **No ROS or hardware layer** in the repo. The only real-robot-shaped code is the examples (live mapping + MPC, reactive control) and an exported C++ runtime for other GPUs (Jetson, relevant later for a Go2-carried arm). XRDF export exists for Isaac Sim / Isaac Lab.

## 3. What Piper Studio already provides (read in this repo, 2026-11-08)

| For cuRobo | Status |
|---|---|
| URDF with correct kinematics, limits, meshes | Yes. The `piper_arm` macro is generated from the official AgileX model and checked against it. |
| Base frame and tool frame | `piper_base_link`, `piper_tcp_link` (fingertip centre = gripper_base + 0.138 m z, D7). |
| Named poses | `zero`, `ready` (and the internal `park`) in the SRDF; `piper_core.named_poses_from_srdf`. |
| Flat URDF a planner can parse | Probably via `piper_sim/scripts/export_urdf.py` *(not read; verify)*. |
| Dense trajectory execution | `arm_controller` is a standard `joint_trajectory_controller` (FollowJointTrajectory) and takes multi-point timed trajectories; ends at rest as cuRobo trajectories do *(not tested with one)*. `piper_py.move_joints` sends a single segment only. |
| Streaming joint targets (MPC, servo) | `arm_position_controller` at a 200 Hz manager rate. |
| Wrist camera in simulation | ZED X Mini on the ZED wrapper's topic names (since 0.9.0) with `camera_info` and TF, in Gazebo, MuJoCo, Isaac. Real camera driver launch: not done (M8). |
| Real arm through the ROS stack | Never run (C10). Only SDK scripts have moved the arm. |

## 4. What the Piper description must give cuRobo, and what it need not

Status: **have**, **weak** (exists but placeholder or unverified), **missing**.

### Needed

| # | Input | Why | Status |
|---|---|---|---|
| 1 | Kinematic chain (origins, axes, types) | FK/IK/Jacobians | have |
| 2 | Joint position limits | bounds | have. A real Piper rests about 0.04 rad past the limits of joints 2 and 3 (REAL_ARM 6b), so a measured start state can be outside cuRobo's bounds: solve it in the description or the start-state handling, not by adding downstream checks. |
| 3 | Velocity limits | timing | weak: URDF 5 rad/s is a vendor placeholder; the arm stores 3.0 (piper-01.md) |
| 4 | Acceleration limits | timing | weak: arm stores 5.0 rad/s²; the MoveIt yaml copies it |
| 5 | Jerk limits | cuRobo trajectories are jerk-limited | missing: no source; choose conservatively, tune against measured tracking |
| 6 | **Torque limits per joint** | cuRoboV2 torque-limited planning | **missing / wrong**: URDF 100 N·m is a placeholder (D4). Biggest unknown. Sources: vendor spec, or motor current feedback times torque constant. If unobtainable, run with torque limits off and label every result. |
| 7 | **Link inertials** | the RNEA torque check | weak: CAD estimates, total 4.70 kg with gripper, unmeasured (D5). Verify total mass. |
| 8 | Mass of everything on the arm (gripper, camera, stand) | dynamics and collision | weak: camera and bracket inertials are nominal; stand 0.04 kg placeholder. Weigh them. |
| 9 | Payload of the held object | V2-Fast updates it at run time | task-time input, not a description item; check the API *(verify)* |
| 10 | Base mounting and gravity direction in the base frame | gravity enters the torque check | have for table mounting; a carrier (Go2 trunk) tilts it, so pass the mount rotation *(verify how cuRobo takes gravity)* |
| 11 | Collision spheres | self/world collision, swept checks, depth self-masking | missing: cuRobo fits them. Must cover the **camera stand and camera body** (Piper Studio models the stand as a box) and the **fingers at the right width**. Measure coverage and protrusion. |
| 12 | Self-collision ignore matrix | correctness and cost | missing: cuRobo computes it; cross-check against the official SRDF disabled pairs |
| 13 | Gripper handling: finger joints locked at a chosen width (as the Franka config does) | cspace = the 6 arm joints | decide. Real stroke is 70 mm stored vs 0.1 m in the URDF (D6, open); the locked width and finger spheres depend on it. |
| 14 | Tool frames | goals are given for a tool frame | have |
| 15 | Retract / default pose | seed and null space | have (`ready`, `zero`) |
| 16 | Joint names and order | `JointState` matches by name | have: build the cuRobo config from the prefixed URDF so names equal `/joint_states` |
| 17 | Camera intrinsics, extrinsics in the wrist frame, optical-frame convention, timestamps | mapper pose and robot masking | weak: simulated values are exact; the real mount is nominal (CLEANUP C5 4.5 mm, C9 calibration). Map quality is limited directly by this. |

### Not needed for cuRobo's role

- MoveIt, the SRDF as a planning input, MoveIt Servo, `piper_py` planning calls. (MoveIt only as an independent collision oracle in one test.)
- `command_guard` / `piper_core` re-validation of a planned trajectory: duplicate of what cuRobo does. The existing guard sits at the hardware boundary of the real driver and is cheap at 200 Hz; whether to keep it on the cuRobo path is the user's decision.
- Visual meshes and colours, the ros2_control block, controller yaml, mimic joints (lock the fingers), the `piper_` prefix itself (names only have to match).
- Simulator models as inputs. They are test beds for execution and perception, not sources of the cuRobo model.
- MoveIt-style inflated margins. Set cuRobo's buffers from measurement.

### Correction made during the pass
An earlier suggestion to turn off effort checking was wrong for cuRoboV2. Supply true torque limits and inertials (items 6 to 8), or switch torque off explicitly and say so.

## 5. Changes likely needed in Piper Studio (to confirm with evidence from the study)

Description side: a planner-neutral flat URDF with the fingers fixed at a chosen width and the TCP, stand and camera links (D1); collision geometry for the camera and stand matching what MoveIt checks (D2); one owner for velocity, acceleration, jerk and torque limits with provenance, instead of the URDF placeholder, the MoveIt yaml copy and a third cuRobo copy (D3); the gripper stroke truth (D4); handling of a measured start state outside the limits (D5); optionally a generated spheres file with a `check`, like the macro (D7).
Control side: a client call that executes a timed dense trajectory (`piper_py.execute_trajectory`, and `RealArm.execute`, tracker B3), with no extra checks; documented state rate with valid velocity and timestamp conventions for state and images; measured tracking lag of the real arm using `stepresp` (the firmware smooths `move_j` itself and takes a speed percent, so timing fidelity is unknown; Gazebo is kinematic and MuJoCo's servo is provisional); defined stop/hold behaviour on cancel.
Perception side: camera mount calibration and the 4.5 mm ambiguity become required; a real ZED driver launch with the same topic and frame names.

## 6. Documentation problems found in this repo (not yet fixed)

- `ARCHITECTURE.md` header says migration step 6 is open, its table says done; step 2 says "Isaac not yet run", disproved by later passes.
- `ARCHITECTURE.md` interface table says `RealArm.execute` implements cuRobo trajectories; it is only planned (B3). `piper_py` has no dense-trajectory call.
- `CLEANUP.md` B7 points to `comms/FOR_OTHERS.md`, archived in A7.
- Stroke inconsistency (D6) and placeholder limits (D3, D4) matter more now that a planner consumes them.

## 7. Staged test plan (simulation only until the last step)

0. cuRobo baseline on its bundled Franka through `gpu-run` (needs approval to install the env).
1. Build the Piper model from this repo's expanded description: camera and stand included, fingers locked; FK parity with the URDF (target 1e-6 m), limits equal, sphere coverage metrics and a visual check, ignore matrix vs the official SRDF.
2. Collision agreement: random configurations, cuRobo against an independent check (MoveIt `GetStateValidity` on a mock launch and/or mesh-mesh on the collision STLs). Spheres must be conservative.
3. Planning: reachability, obstacles, start states `zero`, `ready`, torque and payload on/off, latency.
4. Execution in simulation: dense trajectory to `arm_controller` on mock, then MuJoCo; executed vs planned, then a limit sweep.
5. Perception: sim ZED X Mini with TF pose and joint state into `Mapper` and `RobotSegmenter`; verify the optical-frame convention on a known scene; sensitivity of map error to a wrong mount offset; ESDF resolution vs the small arm.
6. Closed loop in simulation on `scene_objects.yaml`; optionally MPC through the streaming controller.
7. Real arm: only with the user at the arm, after measured tracking, true limits and camera calibration.

## 8. Caveats and open decisions

- An earlier bake-off (`ml_tests/experiments/pick_pipeline_bakeoff`) already built a Piper cuRobo config and logged planning results. It was **not relied on** and not reviewed. What I saw in its config file: built from the unprefixed vendor URDF, finger joints fixed at 0 (closed), one or a few spheres on several links, no camera or stand, acceleration 10 instead of the arm's 5. Re-derive everything.
- Open for the user: where true torque limits come from; whether the cuRobo model bundle eventually moves into this repo; whether to fix the doc problems in section 6 now; env install approval.
- Not read: `export_urdf.py`, camera collision geometry in the camera xacro, what `agx_arm_ctrl` publishes for joint velocity, cuRobo's dynamics code (gravity, friction), the YAM deployment details in the papers.
