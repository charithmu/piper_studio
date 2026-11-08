# Architecture: one Piper, several ways in

Status: target design agreed 2026-11-08; migration steps 1-5 are done (table at the end), step 6 (example projects) is open.

## Goals

1. **Standalone.** This repo alone runs the Piper in simulation (mock, Gazebo, MuJoCo, Isaac) and on the real arm.
2. **Importable.** Other projects (ML endpoints, a Go2 carrying the arm, cuRobo-style planners) use all of it or only a part,
   with their own simulator or none, and without ROS if they want.
3. **Thin wrappers.** One implementation of each rule (limits, guard, named poses, firmware handling); every interface is a thin
   wrapper over it, so the ROS, non-ROS and MoveIt paths cannot drift apart.
4. Simulators, scenes and demos stay in this repo for our own use. They are never part of what others import.

## Repository map and responsibilities

One rule decides where code goes: **every rule or fact has exactly one owner; everything else is a thin wrapper that calls the owner.**
Exported parts never import internal ones. Dependencies below are the ones declared in each `package.xml` (checked 2026-11-08).

```
piper_studio/
├── external/agx_arm_ros/   VENDOR (git submodule, unmodified): official AgileX stack = agx_arm_description (URDF + meshes),
│                           agx_arm_moveit (SRDF), agx_arm_ctrl (the real-arm driver node), agx_arm_msgs. Built by colcon with our packages.
├── src/                    OUR colcon packages (below)
├── isaac/                  Isaac Sim side: runner, URDF->USD conversion, model check. NOT a colcon package: runs in Isaac's own Python env
├── tools/                  scripts for humans: real_arm/ (direct SDK, no ROS), demo/ (demo page), capture/ (stills, video)
├── scripts/                shell entry points: bootstrap, session/env, isaac.sh, run_demo.sh, test_all.sh, can_up.sh, reference_repos.sh
├── env/                    pinned Python environment (uv): requirements.in/.txt
├── docs/                   this design, the contract, the tracker, versions, decisions, real-arm knowledge, archive/ (historical)
├── for_reference/          (git-ignored) cloned vendor/community repos for reading; scripts/reference_repos.sh restores them
└── build/ install/ log/ .venv/   generated, git-ignored
```

| Package | Owns (the one place for...) | Depends on (declared) | Exported? | ROS? |
|---|---|---|---|---|
| `piper_description` | the robot model: `piper_arm` macro (generated from the official model), tool frames, camera models and mounts, ros2_control block, meshes; name prefix; `robot_description()`; validation of the generated files | `agx_arm_description` | **yes** | xacro only |
| `piper_core` | the rules: joint limits from the URDF, the command guard, named poses (later: the direct real-arm driver) | nothing | **yes** | **no** |
| `piper_control` | the control stack: controllers (one config for all backends), MoveIt and Servo config, `command_guard` node (the only prefix translator for the real driver), `control.launch.py`, `launch_lib` builders | `piper_description`, `piper_core`, `agx_arm_moveit`, `agx_arm_ctrl` | **yes** | yes |
| `piper_py` | the client: state, trajectories, streaming, gripper, MoveIt, Servo, camera, CLI; talks only over ROS topics/actions/services, so it needs no other Piper package | ROS messages only | **yes** | yes |
| `piper_sim` | simulators: Gazebo/MuJoCo/Isaac models derived from the description, scenes, `piper.launch.py backend:=...`, export for Isaac | `piper_description`, `piper_control` | no (internal) | yes |
| `piper_examples` | proof of the contract: a carrier robot using the exported parts with its own controller manager | `piper_description`, `piper_control`, `piper_py` | no (internal) | yes |

How the two non-package folders fit in:

- **`external/agx_arm_ros`** is the vendor's code and our source of truth for geometry. We never edit it. `piper_arm.xacro` and `piper_official.srdf.xacro` are generated from its files and validated against them
  (`piper_description/tools/piper_arm_macro.py`); meshes are still loaded from its `agx_arm_description` package; `agx_arm_ctrl` is the driver `piper_control` starts for `backend:=real`
  (the guard sits between ros2_control and the driver). A submodule bump fails the tests until the generated files are regenerated and reviewed.
- **`isaac/`** exists outside colcon on purpose: Isaac Sim ships its own Python and ROS bridge, and must not mix with the apt ROS and `.venv`. `piper_sim` exports the same description to a URDF
  (`export_urdf.py`), `isaac/convert_urdf.py` turns it into USD in `~/data/ml/isaac/piper_studio/`, and `isaac/run_piper.py` runs it as a separate GPU process that exchanges joint states, commands, `/clock` and camera images
  with the ROS side over topics, exactly like the real driver does. `scripts/isaac.sh` is the only entry point (it runs through `gpu-run`); `piper_sim`'s launch starts it for `backend:=isaac`.
  `servo_gains.py` keeps the Isaac servo gains equal to MuJoCo's (a test enforces it); `check_model.py` compares the USD with the URDF kinematics.

Dependency direction (arrows read "may use"): `piper_sim`, `piper_examples` -> `piper_control` -> `piper_core`, `piper_description` -> `external/agx_arm_description`; `piper_py` stands alone (topics only);
`isaac/` talks to the ROS side only through topics. Nothing exported imports `piper_sim`, `piper_examples`, `isaac/` or `tools/`.

## Layers

```
 exported (others import these)                                   internal (this repo only)
 ───────────────────────────────                                  ──────────────────────────
 piper_description   URDF/xacro macros, meshes                    piper_sim       Gazebo/MuJoCo/Isaac models, scenes, backend launch
                     (no simulator code)                          piper_examples  carrier robot example (the contract, tested)
 piper_core          pure Python, no ROS: limits, guard,          isaac/          Isaac Sim runner (own environment, over topics)
                     named poses (later: direct real-arm driver)  tools/          real-arm scripts, demo, recording
 piper_control       ROS 2: controllers, MoveIt, Servo,
                     command_guard, control.launch.py
 piper_py            ROS 2 client API + `piper` CLI
```

### Interfaces (the same arm, chosen by what the user needs)

| Interface | Needs ROS | Use | Implemented by |
|---|---|---|---|
| Joint trajectories, direct | no | cuRobo and other planners that produce joint trajectories | `piper_core.RealArm.execute(trajectory)` (real arm), `piper_core` guard on every point |
| State and gripper, direct | no | scripts, logging | `piper_core.RealArm` (wraps pyAgxArm and the firmware-profile handling now in `tools/real_arm/arm_common.py`) |
| Streaming joint targets | yes | teleop, policies | `arm_position_controller` via `piper_py` |
| FollowJointTrajectory | yes | scripted motion | `arm_controller` via `piper_py` |
| Planning and Cartesian | yes | collision-aware moves | MoveIt via `piper_py` |
| Servo | yes | TCP jogging | MoveIt Servo via `piper_py` |
| Vendor-native | yes/no | firmware features | `agx_arm_ctrl` / pyAgxArm, not wrapped |

Rules for the shared parts:
- **Limits come from the URDF.** `piper_core` parses them from the description's URDF; nothing else defines them.
- **The guard is one function** in `piper_core` (finite, inside limits with margin and slack, fresh feedback, bounded step).
  The ROS `command_guard` node and `RealArm` both call it.
- **Safety defaults are not weakened by a wrapper:** motors not enabled at start, explicit speed, named poses `zero` and `rest`.
- **A user's own simulator** only needs `piper_description` (`hardware:=` selects the ros2_control plugin, or none), and, if they
  want our controllers or MoveIt, `piper_control` on top of their controller manager.

### Mounting and prefix (decision D12)

`piper_description` exposes the arm as the macro `piper_arm(parent, prefix, xyz, rpy, gripper)`, so it can be mounted on a Go2
trunk or placed in any scene. Every link, joint and frame name carries `prefix` (default `piper_`; `""` gives the official names),
so the arm cannot clash with a carrier robot's names. The pieces are separate macros a carrier includes as it needs:
`piper_arm`, `piper_tool_frames`, `piper_camera` (a camera model from `cameras/` on a mount from `mounts/`), `piper_ros2_control`.
`piper.urdf.xacro` is the standalone assembly (the arm at the world origin).

The official AgileX URDF has no prefix and is rooted at `world -> base_link`. **Option A (chosen), done differently from first
planned:** the macro is generated once by `tools/piper_arm_macro.py` and *committed* (not built on the fly), with the source commit,
date and file hashes in its header. `check` validates it against the official files (links, joints, limits, inertials, meshes, mimic;
with and without prefix, with and without gripper) and runs in the tests, so a submodule bump fails until `generate` is rerun and the
diff reviewed. The same tool generates the prefixed official SRDF (`piper_control/config/moveit/piper_official.srdf.xacro`). The
real driver keeps the official names: `command_guard` translates the prefix at that one boundary. Config templates
(`controllers.yaml`, MoveIt yaml, MuJoCo inputs) use a `${prefix}` token that the launch fills in (`piper_description.render_config`).
The two options not taken are recorded so we can change later:

| | Option | Why not now | Switch when |
|---|---|---|---|
| A | Generate the macro from the unmodified official URDF at build time | chosen: one source of truth, official updates flow in | |
| B | Fork the official URDF into this repo and edit it by hand | we would own every vendor update and could drift from the official geometry | the generator becomes hard to maintain, or AgileX changes the file structure |
| C | Keep no prefix; only a mount transform | cheapest, but two arms cannot coexist, and names can collide with the carrier robot's | the Go2 integration does not need a prefix after all |

## Migration (each step ends with the tests passing)

| # | Step | Status |
|---|---|---|
| 1 | Manifest dependencies, camera-mount regression test, this document | done 2026-11-08 |
| 2 | `piper_arm` macro with prefix and mount (option A, committed and validated), camera model/mount split, prefix through controllers, MoveIt, MuJoCo, Isaac runner, guard and `piper_py` | done 2026-11-08 (Isaac not yet run) |
| 3 | `piper_core`: limits, guard, named poses done 2026-11-08 (`command_guard` uses it); `RealArm` (from `tools/real_arm/arm_common.py`) open: it moves the real arm | partly |
| 4 | `piper_sim` split from `piper_bringup` and `piper_description` (simulator models, scenes, `backend:=` launch, export script) | done 2026-11-08 |
| 5 | `piper_control` (renamed from `piper_bringup`): `control.launch.py backend:=mock|real|external` runs on any controller manager; `piper.launch.py backend:=` is in `piper_sim` | done 2026-11-08 |
| 6 | `docs/INTEGRATION.md` (contract: names, units, controllers, frames) plus an example project using its own simulator and one without ROS | |
