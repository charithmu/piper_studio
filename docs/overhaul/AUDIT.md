# Source audit and validation limits

Date: 2026-10-08. This is a source/configuration audit. No simulator, build, or real-arm execution was performed during preservation/design.

## Findings in the superrepo's original checkout

| Finding | Evidence / consequence |
|---|---|
| Gazebo uses obsolete MoveIt package | `src/agx_arm_gzsim/package.xml` and MoveIt launch reference `piper_with_gripper_moveit`; checked-out vendor package is `agx_arm_moveit` |
| Gazebo description include absent | `piper_with_gripper_realsense_description.xacro` is not in the pinned description tree |
| MuJoCo description include absent | `agx_arm_with_tcp.xacro` is not in the pinned description tree |
| Wrapper/launch argument mismatch | Wrapper passes `backend`; motion launches read `profile` |
| Execution outcome ignored | `MotionFacade` calls execute then returns success after successful planning |
| Multiple planning instances | Facade embeds MoveItPy while documented launch also starts move_group; scene ownership needs a deliberate design |
| Latest transform used for stamped observations | Motion goal transform looks up time zero rather than observation time |
| Vendor trajectory path uses mock hardware | Official configuration has GenericSystem and forwards controller output to SDK bridge; measured-state tracking is a separate concern |
| Camera integration still scaffolding | Bringup only logs a placeholder; camera macro is empty |
| Package/roadmap state differs | Historical docs describe simulation-only motion, while working code adds real profiles; proposed storage and package boundaries conflict |
| Relocated environment has old paths | Venv activation targets the pre-move workspace; installed overlay reports multiple missing local_setup.bash files |
| Source preservation incomplete | MuJoCo was an unregistered nested repository; staged camera URL did not resolve; local branch tips contained unpushed changes |

## Important correction after examining all local branch tips

The original checkout does not include all previously committed development. In particular:

- ROS local `ros2` tip `6d7ec47` includes feedback/control topic fixes and TCP/visualization changes.
- Gazebo local `main` tip `0ff5333` includes a three-commit integration update covering dependencies, gripper configuration, and TCP/initial positions.
- Detection, camera, graspgen, and manipulation each have a later package-maintenance commit on their local `main` branches.
- MuJoCo `95f6d89` contains its unpushed TCP/initial-position update.
- ROS tip `6d7ec47` references description commit `f564787`, which introduces the missing top-level TCP xacros. That commit had no branch/ref in the description repository; it is now preserved in a local branch and checked-in bundle/patch.

These commits must be examined before reimplementing fixes. Their presence does not prove runtime behavior or compatibility with October upstream. The archive preserves both the current snapshot and the additional branch tips without merging them together.

## Model differences

Parsed explicit mass sums include the fixed base and gripper; they are model definitions, not measured hardware mass.

| Property | Pinned ROS model | Local MuJoCo model |
|---|---:|---:|
| Explicit mass sum | 4.660 kg | 2.3669165 kg |
| Joint 3 | [-2.9670597, 0] rad | [-2.697, 0] rad |
| Joint 4 | +/-1.7453292 rad | +/-1.832 rad |
| Joint 6 | +/-2.0943951 rad | +/-3.14 rad |
| Jaw travel | 50 mm per jaw | 35 mm per jaw |
| Gravity compensation | No corresponding per-body setting | `gravcomp=1` on every body |

The MuJoCo model also includes simplified inertias and independently actuated jaws. Validate geometry, limits, gripper constraints, and inertial assumptions before using it for actuator identification or sim-to-real claims.

## Upstream facts

Verified candidate heads are in [official-candidates.json](official-candidates.json). Against the original pins, ROS has 19 newer commits, SDK 35 upstream commits plus one local exclusion commit, and the description 7 newer commits. The handoff's 76-commit SDK estimate is not the comparison observed in this audit.

The June aperture-joint migration, later flange/gripper mounting revisions, SDK firmware resolver dependencies, and MIT torque conversions make a coordinated migration necessary. Not every latest change targets standard Piper; Nero fixes must not be generalized to this hardware.

## Checks performed during preservation

- 804 archived regular working files passed SHA256 comparison; 13 original Git history bundles verified.
- A supplemental 1,272-byte bundle preserves the otherwise unreferenced TCP-description commit and verifies against its parent `3080af4`.
- Python AST parsing passed for 25 Python files across motion, detection, camera, MuJoCo, and workspace packages.
- Five package XML files parsed; launcher passed `bash -n`.
- A pattern scan of 168 small candidate source/document files found no matching private keys or common token formats. This is a limited check, not a claim that automated secret detection is exhaustive.
- Git recognizes all declared superrepo submodules, including MuJoCo and the renamed wrist-camera package.

No colcon build, launch, controller, CAN device, GPU job, dependency install, or physical test was run. The existing installed overlay emitted missing-path warnings when sourced for static checks. Archive labels deliberately avoid claiming a working release.
