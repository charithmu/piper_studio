# Implementation roadmap

Status: proposed. Work proceeds one bounded job at a time, with evidence recorded in `comms/STATUS.md` and an active acceptance log outside ignored runtime build logs. No parallel agent rollout is required by this proposal.

The archive is historical preservation. A later integration branch, proposed as `refactor/official-stack-20261008`, should implement the agreed design. Keep legacy branches available until the new profiles pass.

## Phase 0: preserve and publish

Done locally: backup, package commits, archive refs, corrected camera remote mapping, MuJoCo gitlink registration, and separate historical-planning commit.

Remaining: publish approved child refs, verify their remote SHAs, publish parent archive/planning branches, verify fresh checkout/submodule accessibility. Preserve the unpublished TCP description in the checked-in bundle and patch. No force-pushes, main-branch updates, resets, or GitHub repository renames.

Acceptance: no uncommitted source changes; all archive refs reported; all referenced package commits remotely reachable; unpublished legacy description recoverable from the bundle.

## Phase 1: reconcile history and choose the official compatibility set

First implementation job, approximately 30–45 minutes for inspection and the migration matrix, excluding downloads/builds.

- Compare the later saved ROS/Gazebo/package branch tips with the current snapshot and official sources.
- Classify every local patch as upstreamed, still required, obsolete, or experimental; do not merge older branch tips indiscriminately.
- Verify official revisions again and record a candidate SDK/ROS/description compatibility set.
- Create dedicated integration branches with minimal local patches. Retain `COLCON_IGNORE` or an equivalent workspace exclusion for the SDK.
- Review the June aperture-joint migration, flange/mount changes, firmware resolver dependencies, and MIT torque scaling.

Acceptance: reviewed patch matrix; exact candidate source SHAs; no missing model includes in the proposed source graph; no firmware or robot commands.

## Phase 2: reproduce the base environment and canonical description

Separate bounded jobs for environment setup and model composition. Workspace installs may proceed only within their approved scope; system/shared-environment changes require approval.

- Create an isolated integration checkout and a new project environment. Retain the old environment/build products.
- Source Jazzy, the correct workspace environment, and its overlay in order.
- Capture ROS/Python/OS versions and import/ABI compatibility. Exclude optional simulators and learning frameworks from the base environment.
- Build the official description/messages/control/MoveIt configuration and the workspace composition narrowly.
- Compose the standard Piper/gripper, flange/TCP, and optional camera mount from official assets.
- Generate a model report for kinematics, limits, gripper semantics, masses/inertias, and provenance.

Acceptance: fresh isolated model builds/expands; no stale absolute paths; coherent frames and aperture mapping; model report has no unsupported claims about physical limits.

## Phase 3: implement the control boundary and mock profile

Implement only the contracts needed for the first robot and tested backends.

- State/command/result/capability records, resource ownership, finite goal lifecycle, and streaming freshness.
- Mock adapter and ROS adapter, with MoveIt-independent joint/gripper control.
- Correct execution results, cancellation/preemption, timestamped transforms, and parameterized instance names.
- Default MoveIt client/scene authority; optional embedded profile remains explicit.
- Evaluate topic-based measured-state ros2_control integration while retaining the official SDK.

Acceptance: failed execution cannot report success; conflicting command owners are rejected; stale feedback/chunks and unsupported modes are handled; direct-control profile works without MoveIt imports.

## Phase 4: MuJoCo

- Qualify the maintained ros2_control adapter and a pinned model/engine version.
- Reconcile limits, gripper coupling, inertial assumptions, and gravity compensation.
- Record conversion and model refinements; separate ideal gravity-compensated experiments from the nominal model.
- Run headless state, gripper, trajectory, cancellation, and frame tests.

Acceptance: commands use the common semantics; joint/TCP tracking and model equivalence are reported; dynamic assumptions and failures are visible.

## Phase 5: Gazebo Harmonic

- Reuse the reconciled historical fixes where still applicable.
- Build the current official description into a Gazebo overlay, with maintained gz_ros2_control integration.
- Align controller resources, aperture semantics, frames, and clock behavior.
- Run the same functional scenarios used for MuJoCo.

Acceptance: reproducible headless launch and scenario results; no obsolete MoveIt package dependency; no duplicate TF/state/controller ownership.

## Phase 6: Isaac Sim and Isaac Lab

Coordinate the GPU window through the existing status files. Use `gpu-run`; do not install into the shared environment.

- Qualify the current official Piper USD against the canonical description and importer settings.
- Prefer the installed native `isaacsim.ros2.control` extension for the ROS profile; establish one controller manager and robot-description authority.
- Test USD drive/interface mapping, mimic gripper behavior, trajectory execution, and clock/reset behavior.
- Keep direct Isaac Lab batched environments as a separate learning profile using the same semantic specifications and explicit actuator assumptions.

Acceptance: ROS profile passes the common scenarios; asset conversion is repeatable; direct Lab and ROS interfaces do not get conflated; installation presence is not substituted for Piper integration evidence.

## Phase 7: actual camera and perception/task workflow

Requires camera identity and mount information from the user.

- Wrap the actual vendor camera driver; add simulator and replay adapters.
- Define timestamped CameraInfo/image/depth/optical-frame contracts and calibration identity.
- Implement one deterministic detection-to-grasp workflow; evaluate standard detector components and MoveIt Task Constructor.
- Add additional cameras after the first complete workflow passes.

Acceptance: observation-time transforms are correct; calibration assets are versioned; sim/replay observations exercise the same downstream code.

## Phase 8: data collection, teleoperation, policies, and agents

- Record commands, measured states, results, camera samples, synchronization, controller/firmware/model metadata, and episode boundaries.
- Add a versioned learning-framework exporter and a policy runner with declared action specifications.
- Exercise action chunks, replay, cancellation, timeouts, ownership handover, and observation latency in simulation.
- Evaluate external teleoperation/LeRobot examples without replacing the entire workspace with their dependency stacks.

Acceptance: replayable episodes with complete provenance; joint and Cartesian policies can select supported control levels; incompatible actions and concurrent writers are rejected.

## Phase 9: supervised hardware qualification and platform composition

Requires the user's physical presence and approval of each hardware step. No firmware update is part of this roadmap.

- Read status/firmware using an explicitly reviewed path that cannot auto-enable or move the arm.
- Validate position-control startup, state synchronization, gripper travel, low-speed trajectories, tracking, and cancel/fault behavior.
- Qualify MIT/effort modes separately with firmware-specific mappings, measured actuator behavior, and appropriate gains.
- Compose the arm with Go2 through robot-instance frames and mounting transforms; do not copy Piper-L policies or gains into standard-Piper hardware.

Acceptance: recorded hardware qualification for the selected firmware/controller profile; platform integration uses the same contracts and consistent transform ownership.

## Evidence and release gates

Each phase reports exact versions, commands, outcomes, relevant tolerances, and unverified assumptions. Model equivalence, controller tracking, contact/dynamics validity, and hardware qualification are separate claims.

The first supported release requires clean-checkout setup, base/mock/direct-control checks, canonical-model validation, and at least one complete simulator workflow. All three simulator profiles are required for the requested full setup. Hardware and learning profiles receive their own support status; a passing simulator test does not qualify physical impedance control.

Do not retire a legacy workflow or consolidate repository histories until its replacement and recovery path are reviewed.
