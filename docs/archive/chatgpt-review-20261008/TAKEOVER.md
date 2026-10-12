# Piper Studio: full session context and takeover instructions

Prepared 2026-10-08 UTC for a new agent, at the user's explicit request. This records the conversation, actual work, proposed design, evidence, permissions, unresolved questions, and continuation procedure. It supplements the original `HANDOFF.md`; it does not overwrite it or replace the user's current instructions.

**Current state:** legacy source and plans are committed locally; the new overhaul is designed but not implemented. **No push has executed.** Publication is pending exact-destination authorization after automatic approval review rejected the batch push. No build, simulator, dependency install, CAN command, firmware change, or real-arm test has been performed by this agent.

Workspace: `/home/atlasdev/projects/ros/piper_studio`, on Linux workstation `atlas`, account `atlasdev`. User/GitHub identity: `charithmu` (Charith Munasinghe). Git author configuration: `Charith Munasinghe <mung@zhaw.ch>`.

## 1. User instructions and how the scope evolved

### 1.1 Original startup instruction, verbatim

> You are the Piper Studio agent on the Linux workstation "atlas" (account atlasdev). Your working directory is /home/atlasdev/projects/ros/piper_studio. You have no prior context, so before doing anything read these files completely, in this order:
> 1) /home/atlasdev/projects/ros/piper_studio/HANDOFF.md  (self-contained: how we work, machine rules, your mission and task list; follow it exactly)
> 2) this workspace's AGENTS.md, README.md, PLAN.md, TODO.md, ORCHESTRATION.md
> 3) /home/atlasdev/projects/platform/README.md and CLAUDE.md  (machine knowledge base and rules)
> Short version of the rules: report to me after every step with evidence; one bounded job at a time; I am an expert in robotics, so be technical and precise and discuss concepts alternatives and best practices with me.; my working tree here has uncommitted changes, so review it first and commit by working with me first and never push, reset or delete without asking; there is no passwordless sudo, so give me exact commands one at a time with an explanation; ask before installs outside this workspace, network exposure, and anything that moves the real arm (I must be present). There are other agents working on this workstation and some of them are using Claude code. Since You cannot talk to other agents: communicate through the files in comms/ described in HANDOFF.md section 1.4 (read comms/INBOX.md at the start of each session; put questions in comms/NEEDS_USER.md and progress in comms/STATUS.md) and through this chat.
> Your first reply: what you read, your understanding of the mission in five lines, your questions, and the proposed first step (task 0) with a time estimate. Do not change anything until I agree.

The required files were read completely in the requested order. Long output that truncated was reread in bounded chunks. The initial reply summarized the mission, identified dirty-tree questions, and proposed a baseline/backup job; it did not modify files.

### 1.2 The user then changed the immediate objective

The user explained that Piper Studio was older personal development gathering SDKs, ROS packages, descriptions, and simulators. New vendor/community developments might make some work redundant. He requested internet research, deep analysis of the organization, an expert opinion, and a path to update the master workspace.

Hardware scope is **normal/standard Piper with the normal AgileX gripper**, not Piper-L or another variant. The user explicitly clarified that the original handoff was written by another agent to establish machine/workflow context; its task list and architectural assertions are not an authoritative design. Better alternatives should be proposed. Decisions must be communicated before irreversible actions, and progress must remain reliable and visible. The user has SSH and physical access.

This agent therefore conducted a read-only audit and upstream research first. No baseline simulator/hardware launch was attempted merely because the older handoff listed one.

### 1.3 The user authorized preservation commits and pushes, then requested design

The user's subsequent direction was to commit everything, push, preserve older implementation work separately, preserve newer planning documents on new branches, and design the overhaul before refactoring/building. His intended system must support:

- Multiple simulation environments and real hardware.
- Hierarchical **control** facades, including lower levels than planning.
- Traditional planning with MoveIt, operation without MoveIt, and algorithm experiments.
- Data collection, VLA policies, teleoperation, and agent-driven workflows.
- Other robot platforms and multiple camera types.
- Official current dependencies/descriptions wherever practical, firmware compatibility, reliability, extensibility, and maintainability.

This authorized the local preservation commits and a push attempt. It did not authorize firmware changes, physical movement, shared/system installs, or an unreviewed repository-history consolidation. Architecture implementation has not started.

### 1.4 Latest instructions

The user provided replacement `AGENTS.md` instructions, explicitly superseding earlier versions. The current on-disk [AGENTS.md](AGENTS.md) matches those instructions, including corrected `/projects/dev/` environment paths and preservation/overhaul planning rules. Read it completely.

The latest task is to write this complete takeover file. **That request is not approval of the pending push destinations or the proposed architecture.** No reply to the exact-destination question has arrived.

## 2. Working agreement and machine boundaries

- Discuss robotics/control choices as with an expert colleague. Explain unfamiliar Isaac/RL concepts briefly when needed. Report each bounded step with evidence and limitations; do not leave the user without meaningful progress updates.
- Read [comms/INBOX.md](comms/INBOX.md) at session start. Append progress to [comms/STATUS.md](comms/STATUS.md), questions/required commands to [comms/NEEDS_USER.md](comms/NEEDS_USER.md), and useful integration facts to [comms/FOR_OTHERS.md](comms/FOR_OTHERS.md). State UTC or local time. Update status before reporting completion.
- Do not use direct agent channels. No subagents were spawned. `ORCHESTRATION.md` describes an older multi-agent proposal; any actual rollout still requires its human checkpoints and current authorization.
- Preserve working-tree changes and histories. No reset, force-push, deletion, or repository rename has been authorized/performed here. Main/master/ros2 branches were left unchanged.
- There is no passwordless sudo. Provide exact commands to the user one at a time, with purpose and expected evidence, when root is needed.
- Ask before installs outside the project, changes to shared environments, service/network exposure, credentials/firewall/SSH changes, and anything capable of moving the arm. The user must be physically present and approve the exact hardware step. No firmware upgrade is authorized.
- Use `uv` for project environments/packages. Do not install into the system Python or another project's environment. This agent installed nothing.
- Source ROS before any ROS command/build/test/launch, then the workspace venv, then its overlay when present:

```bash
source /opt/ros/jazzy/setup.bash
source /home/atlasdev/projects/ros/piper_studio/.venv/bin/activate
source /home/atlasdev/projects/ros/piper_studio/install/setup.bash
```

- ROS is Jazzy; Gazebo is Harmonic. The existing venv activation still points internally to the old path without `/dev`, and installed overlay files have missing-path problems. Sourcing these is not proof they work. For read-only Python checks this agent activated the environment then explicitly used `.venv/bin/python` with `PYTHONDONTWRITEBYTECODE=1`.
- Metadata snapshot rechecked during this takeover task: pyAgxArm 1.0.0, MuJoCo 3.6.0, robot_descriptions 2.0.0, NumPy 2.4.3, SciPy 1.17.1, python-can 4.6.1, colcon-core 0.21.3. The environment permits system-site packages for ROS integration. These versions describe installed metadata, not successful imports, binary compatibility, or a reproducible dependency lock.
- Keep the SDK excluded from colcon using `COLCON_IGNORE` or a subsequently agreed equivalent. Vendor SDK/ROS source is upstream-frozen without coordination; this session made no functional source edits there.
- Large data/models/recordings belong under `/home/atlasdev/data/ml/` or `/home/atlasdev/extra`, not in the project. New data folders require owner/source notes. Do not commit credentials or datasets.
- Atlas has a shared RTX 4090. Read the status boards before GPU work, check `nvidia-smi`, and use `gpu-run`. No GPU work was started here.
- Read `/home/atlasdev/projects/platform/README.md` and `CLAUDE.md` for machine details. Machine/environment changes require a dated knowledge-base entry; obtain permission if that external write is outside your tool scope.
- This session could write the workspace and `/tmp`; Git metadata mutations/network operations required tool escalation. Future agents must check their own permissions. Do not bypass review rejection or work around a write boundary.
- The preserved `.claude/settings.json` contains agent permissions and denies some Git operations. It has no detected credentials. Do not edit tool settings to bypass the push rejection.

Other workstreams, read-only coordination paths:

- Go2: `/home/atlasdev/projects/ros/go2_studio/comms/STATUS.md` and `FOR_OTHERS.md`.
- Isaac/integration: `/home/atlasdev/projects/sim/robosim/handoff/STATUS.md`.
- Isaac setup: `/home/atlasdev/projects/sim/robosim/docs/SETUP_REFERENCE.md` and `SESSION_LOG.md`.
- Installed native control metadata: `/home/atlasdev/data/ml/isaac/envs/isaaclab/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.control/config/extension.toml`; inspect it read-only and do not modify that shared installation.

Their status files were read. The Isaac thread reports Sim 6.1/Lab 3.0 installation and earlier Piper USD tests; those are **that thread's evidence**, not this agent's qualification of Piper Studio. Camera and mounting measurements remain outstanding. Recheck the boards; machine/GPU state may change.

## 3. What was inspected and researched

Beyond the startup files, this agent inspected `DESIGN.md`, the package graph, `.gitmodules`, the full launcher, relevant package READMEs/manifests/guides, motion facade/server/launch/config code, Gazebo/MuJoCo robot/controller launch files, the MJCF model, wrist-camera/workspace scaffolding, official pinned MoveIt configurations, driver references, environment metadata, and the installed Isaac control extension metadata.

The initial audit was read-only. Research used official repositories/docs plus primary community repository sources; GitHub metadata and `git ls-remote` established actual candidate heads. Shell network access sometimes required escalation; later GitHub API requests hit rate limits. Raw GitHub content and `git ls-remote` remained useful. No repository install or update was performed as part of research.

### 3.1 Audit findings in the originally pinned checkout

| Finding | Files / implication |
|---|---|
| Obsolete Gazebo MoveIt dependency | `src/agx_arm_gzsim/package.xml` and `launch/piper_with_gripper_moveit_gzsim.launch.py` require `piper_with_gripper_moveit`, absent from the checked-out vendor stack, which provides `agx_arm_moveit` |
| Missing Gazebo include | `urdf/piper_with_gripper_gzsim.urdf.xacro` includes absent `piper_with_gripper_realsense_description.xacro` |
| Missing MuJoCo include | `src/agx_arm_mjsim/urdf/piper_with_gripper_mjsim.urdf.xacro` includes absent `agx_arm_with_tcp.xacro` in the original description pin |
| Wrapper routing bug | `scripts/piper_studio.sh` passes `backend:=...`; motion launch files read `profile:=sim|real` |
| False-success risk | `agx_arm_motion/motion_facade.py` ignores execute's result and returns true after planning succeeds |
| Planning ownership ambiguous | Motion embeds MoveItPy while instructions also start move_group; independently maintained scenes/configurations need an explicit owner |
| Observation transform timing | Pose goal transforms request the latest TF instead of the stamped observation time |
| Real controller feedback distinction | Official MoveIt config uses mock GenericSystem and forwards controller output through the SDK bridge; using measured feedback in MoveIt does not by itself make ros2_control trajectory tracking use measured state |
| Unfinished camera/perception | Camera launch prints a placeholder; macro is empty. Detection/grasp/task packages largely contain contracts/scaffolds |
| Documentation drift | Historical docs, working code, proposed calibration storage, and package layout are inconsistent |
| Relocation damage | Venv activation and generated overlay paths still assume the old workspace location |

These defects were **preserved**, not fixed during archiving. Do not claim the archived workspace builds or operates correctly.

### 3.2 Critical later discovery: the root pin hid existing development

Several clean submodules were detached at earlier commits while their local main/ros2 branches already contained later work. The initial audit described the actual pinned checkout; it did not establish that the user had never implemented the missing fixes.

- ROS local `ros2` tip `6d7ec47`: two later commits, including feedback/control-topic and TCP/visualization changes.
- Gazebo local `main` tip `0ff5333`: three later commits, including dependency/gripper integration fixes and TCP/initial-position updates.
- Detection/camera/graspgen/manipulation each have a later maintenance commit.
- MuJoCo `95f6d89` has its unpushed TCP/initial-position update.
- ROS `6d7ec47` points its nested description at previously unreferenced commit `f564787`, adding the missing top-level TCP xacros.

**Before writing replacement fixes, compare these saved tips with October upstream.** They were preserved separately without silently moving the root's pins to them or merging their changes into dirty work.

### 3.3 Model parity findings

These are explicit file values, not physical measurements. Mass sums include the fixed base and gripper.

| Property | Pinned ROS description | Local MuJoCo MJCF |
|---|---:|---:|
| Explicit mass sum | 4.660 kg | 2.3669165 kg |
| Joint 3 | [-2.9670597, 0] rad | [-2.697, 0] rad |
| Joint 4 | +/-1.7453292 rad | +/-1.832 rad |
| Joint 6 | +/-2.0943951 rad | +/-3.14 rad |
| Jaw travel | 50 mm per jaw | 35 mm per jaw |
| Gravity compensation | No corresponding per-body setting | `gravcomp=1` on every body |

MJCF: `src/agx_arm_mjsim/mjcf/piper.xml`. It has simplified inertias, independently actuated fingers, and provisional actuator/gain assumptions. Sharing names/controllers is insufficient for dynamics parity. The old URDF's 100 Nm effort fields must not be accepted as physical joint ratings. Limits, actual gripper travel, firmware, inertias, and actuator behavior need provenance/qualification.

The old camera contract claims TCP exists only in SRDF, but the checked-out MoveIt URDF defines an actual TCP link. Later official flange/mount changes make frame-parent and offset checks necessary.

## 4. Official and community research conclusions

### 4.1 Exact official candidate revisions

Verified on 2026-10-08 with `git ls-remote`; reverify when resuming. These are **research candidates**, not a tested compatibility lock.

| Repository | Branch | Exact candidate SHA |
|---|---|---|
| `agilexrobotics/pyAgxArm` | master | `841a625f5f4920e776f20b934eb13048b747e6d0` |
| `agilexrobotics/agx_arm_ros` | ros2 | `e4ccec1999279d063c2962030b9437b78b0d0db6` |
| `agilexrobotics/agx_arm_urdf` | main | `983788b58d7eae76511177f768d85877534917fe` |

Against the original pins: ROS has 19 upstream commits, description 7, SDK 35 upstream commits with one local `COLCON_IGNORE` commit. The original handoff's SDK count of 76 is incorrect for this comparison. SDK local commit `cd6f878` has author date 2026-03-16 but committer date 2026-05-12 and parent `a2de842` from May; do not confuse author date with the actual ancestry/snapshot date.

Material migration changes:

- June ROS/description change: one commanded gripper aperture joint `gripper`, with passive mimic fingers. Driver, URDF, SRDF, controller, simulation, and application mappings must migrate together.
- Later descriptions introduce `flange_link`, changed mounting geometry, and shared gripper meshes. Recheck TCP/camera offsets and collision geometry.
- ROS changes depend on SDK firmware-resolution APIs; choose SDK/ROS/descriptions as a compatible set.
- SDK firmware-specific MIT torque encoding/feedback conversion changed, including the September torque-coefficient change. Audit firmware/model mapping before using inherited gains or feedforward scaling.
- The latest Nero acceleration-unit fix does not establish a corresponding standard-Piper change.
- Old official `piper_sdk` directs users to `pyAgxArm`. Prefer the newer official family for the core; keep legacy-dependent examples optional.
- Current official MoveIt still uses mock GenericSystem. A measured-state topic-based adapter is a candidate improvement, not already implemented here.

Source links and candidate JSON: [UPSTREAMS.md](docs/overhaul/UPSTREAMS.md), [official-candidates.json](docs/overhaul/official-candidates.json).

### 4.2 Selective adoption rather than gathering every repository

| Component | Assessment / proposed treatment |
|---|---|
| Official SDK + ROS + URDF | Core foundation; minimal documented local patches; pin a qualified set |
| ros-controls topic-based hardware interfaces | Evaluate measured-state ros2_control while keeping official SDK; Jazzy supported |
| ros-controls mujoco_ros2_control | Reuse maintained simulator/controller plumbing |
| DeepMind MuJoCo Menagerie Piper | Useful assets/contact reference, not calibrated hardware dynamics |
| Official Piper Isaac assets | Reference USDs, but compare with latest canonical description before adoption |
| Isaac native `isaacsim.ros2.control` | Installed extension metadata inspected (0.1.6); hosts controller manager in-process; preferred ROS candidate, Piper integration still untested here |
| AgileX College IsaacLab examples | Task/teleoperation/data examples; selectively reuse after version checks |
| MoveIt Task Constructor / Servo | Optional multistage planning / Cartesian streaming rather than reinventing those layers |
| Renesas Piper C++ hardware/MuJoCo | Isolated alternative evaluation; different protocol implementation, legacy gripper semantics, activation/deactivation behavior |
| justagist/piper_cpp | Native alternative; ROS plugin position commands, defaults include moving to zero on activation; not a firmware-qualified drop-in |
| IIT-DLS stack | Impedance research; default Piper-L geometry/gains cannot be copied to standard Piper |
| LeRobot Piper plugin | Optional learning integration, currently legacy piper_sdk dependency |
| PiperPilot | Experimental pyAgxArm teleoperation/recording; review control modes and dataset format separately |

Isaac native control infers command interfaces from USD drives and can generate/publish a robot description. Prevent duplicate controller managers/description authorities. Its presence or UR10 tutorial is not proof the Piper profile works. Isaac Lab batched policy training is a distinct execution profile, not the same as ROS trajectory control.

No external component above has been newly installed or imported into the workspace during this session.

## 5. Preservation work actually completed

### 5.1 Original dirty state and what was preserved

The root had modified instructions, `.gitignore`, AGENTS/DESIGN/README/TODO, launcher changes, staged submodule edits, a staged camera rename, new workspace/calibration submodules, untracked plans/graphs/handoffs/communication files, `.claude/settings.json`, and an unregistered standalone MuJoCo repository. Motion, detection, and camera submodules were dirty.

User deletions included motion's old AGENTS.md and root `log/checkpoints.md`; the latter contained only a template. They were preserved in the archive rather than restored speculatively. Root `.gitignore` now includes `log/**` and `for_reference/**`. The active AGENTS/ORCHESTRATION still refer to checkpoint logging; resolve durable acceptance logging with the user before a real rollout, and do not silently abandon the human checkpoint rule.

### 5.2 Verified backup

`/tmp/piper-studio-preservation-20261008T153911Z` exists at handoff time.

- 13 Git history bundles verified, including original HEAD/ref histories.
- Original index/config/ref metadata and staged/unstaged binary patches saved per repository.
- 804 non-ignored working files archived; every regular file's SHA256 compared with the archive.
- Initial backup total: 247,031,510 bytes, before supplemental model bundle/restore-check additions.
- Inventory: `inventory.json`; working-file hashes: `file-sha256.json`; source note: `SOURCE.md`.
- Archive-ref inventory: `preserved-branches.json`.
- No `published-branches.json` exists because publication did not run.
- Runtime venv/build/install products are excluded. `/tmp` is temporary; the durable Git publication remains unfinished.

### 5.3 Superrepo commit history

| Commit | Purpose |
|---|---|
| `1bcd80e73cd1903cf095f63b80337e574ac3b3a5` | Original main; unchanged |
| `f6aa65b6969ea1ba558948347e9ef5294bcdc896` | Legacy implementation snapshot |
| `2f5f491` | Historical plans, package graph, handoffs and initial comms saved separately |
| `fe567e8` | Previously unpublished TCP-description bundle/patch and restore instructions |
| `4b342ba` | Full new architecture/audit/roadmap/provenance proposal; root documentation pointers and corrected AGENTS paths |
| `fd5690dda8193d34f46ba2d891dd51cbd36b58b9` | Final pre-takeover status/validation record |

`archive/legacy-work-20261008` points at `f6aa65b`. Current checkout is `planning/overhaul-20261008`. This takeover document's commit is a later commit on that planning branch; use `git rev-parse HEAD` to obtain it rather than assuming `fd5690d` is still the tip.

### 5.4 Child repository archives

Ten user-owned packages have `archive/legacy-work-20261008`. Six also have `archive/legacy-branch-tip-20261008`, retaining their later existing local main/ros2 commits. Sixteen package refs total; none pushed.

| Package path | Snapshot SHA | Additional saved tip | Remote repository |
|---|---|---|---|
| `src/pyAgxArm` | `cd6f878` | — | charithmu/pyAgxArm |
| `src/agx_arm_ros` | `125ebe2` | `6d7ec47` | charithmu/agx_arm_ros |
| `src/agx_arm_gzsim` | `735e076` | `0ff5333` | charithmu/agx_arm_gzsim |
| `src/agx_arm_motion` | `235fdd6` | — | charithmu/agx_arm_motion |
| `src/agx_arm_detect` | `20678a3` | `20f0e7d` | charithmu/agx_arm_detect |
| `src/agx_arm_wristcam` | `3a4d689` | `fac10b0` | charithmu/agx_arm_eyes |
| `src/agx_arm_graspgen` | `4f3b99b` | `4b29e09` | charithmu/agx_arm_graspgen |
| `src/agx_arm_manipulation` | `7b814f4` | `16dc98b` | charithmu/agx_arm_manipulation |
| `src/agx_arm_workspace` | `2164dd2` | — | charithmu/agx_arm_workspace |
| `src/agx_arm_mjsim` | `95f6d89` | — | charithmu/agx_arm_mjsim |

Motion/detection/camera snapshot commits are new preservation commits; the other snapshot refs point to previously existing commits. Full SHAs/remotes are in [preservation.json](docs/overhaul/preservation.json).

Two source-retrieval corrections were included in the legacy root commit:

1. Registered existing MuJoCo as a gitlink/submodule in `.gitmodules`.
2. Corrected the renamed camera package's URL to the accessible `charithmu/agx_arm_eyes` repository. `charithmu/agx_arm_wristcam` returned Repository not found from this identity. The ROS package/directory remains `agx_arm_wristcam`; no GitHub repository rename was performed.

Root local submodule URL settings were updated for those mappings. MuJoCo retains its existing standalone `.git` directory; it was not absorbed/moved. All 11 top-level gitlinks have `.gitmodules` mappings; recursive status recognizes them.

Unchanged external pins: nested official description `3080af4b579238c850b709c411abdc88fc930c82`; MoveIt calibration `3f9d48ebe843caf1de060bfafe78160585c7c26f`. Do not push to their official remotes.

### 5.5 Recovered orphan description commit

Exact commit: `f56478761ebbb4e038270fec1e3f6f760f83a131`; parent `3080af4b579238c850b709c411abdc88fc930c82`. It adds `agx_arm_with_tcp.xacro` and `agx_arm_with_tcp.urdf.xacro`.

It previously had no ref in the nested description repository. A local `archive/legacy-tcp-description-20261008` branch was created. The exact commit is now preserved in:

- [Git bundle](docs/archive/legacy-tcp-description/agx_arm_urdf-tcp.bundle), 1,272 bytes.
- [Readable patch](docs/archive/legacy-tcp-description/agx_arm_urdf-tcp.patch), 3,317 bytes.
- [Restore instructions](docs/archive/legacy-tcp-description/README.md).

Bundle SHA256: `5bb8297a657dc0b099391087b137067b95ec5beec62bfee718286aba5b6334d4`. Verification passed. Recovery into `/tmp/piper-studio-preservation-20261008T153911Z/description-restore-check`, cloned from the original backup, recovered the exact commit and both file blobs.

The basic archive root references official `3080af4` and needs no special model restore. Inspecting the later saved ROS tip with its nested description requires the bundle restore in an isolated checkout. Do not expect a plain submodule update against the official remote to retrieve this private experiment automatically.

### 5.6 Validation evidence and limits

- 25 Python files AST-parsed across motion/detection/camera/MuJoCo/workspace packages.
- Five package XML files parsed; launcher passed `bash -n`.
- Limited private-key/common-token pattern scan over 168 small candidate files found no matches; not exhaustive security validation.
- JSON records parsed and 15 local documentation links checked before this takeover file.
- All 11 top-level gitlink mappings validated.
- Description bundle recovery verified exactly, not just syntax-checked.
- All 13 repositories were clean at the start of this takeover task. Child HEADs match the snapshot table; vendor description remains detached at `3080af4`, vendor ROS at `125ebe2`, Gazebo at `735e076`. Dirty packages now check out their archive branches; other original branch/detached states were retained.

No colcon build, package install, launch, simulation, CUDA job, controller execution, CAN access, or physical test occurred. The existing install overlay emitted missing local_setup.bash warnings for several packages when sourced during static checks. Do not broaden the claims beyond the evidence.

## 6. Push blockage and exact continuation conditions

The user explicitly requested commit and push. A network push script was prepared to push the sixteen new archive refs to the ten existing user-owned package forks, verifying each remote SHA before publishing the parent. It would not update main/master/ros2 or force-push.

Automatic approval review rejected it **before execution**. Its stated reason was:

> This pushes private source history and working-state commits to multiple GitHub remotes; although pushing was requested in general, the transcript does not explicitly authorize these exact destinations or establish that all remotes are user-owned trusted repositories.

No safer substitute was used to bypass the rejection. An explicit destination question was asked in chat and recorded in [NEEDS_USER.md](comms/NEEDS_USER.md). No answer has arrived. The current handoff request is not that answer.

Pending destinations, all existing repositories under `charithmu`:

```text
piper_studio
pyAgxArm
agx_arm_ros
agx_arm_gzsim
agx_arm_motion
agx_arm_detect
agx_arm_eyes
agx_arm_graspgen
agx_arm_manipulation
agx_arm_workspace
agx_arm_mjsim
```

Once authorization is explicit, use the normal reviewed tool path. Publish only the new archive/planning branches, child repositories first. Inspect actual remotes before pushing: some origin URLs are HTTPS, others SSH; the prepared script used explicit `git@github.com:charithmu/<repo>.git` destinations, including `agx_arm_eyes` for the camera. Do not publish the nested description or calibration to official vendor remotes.

Verify remote SHAs with `git ls-remote`, then root archive/planning branches, then source retrieval in an isolated clean checkout. Update preservation JSON/status and report real links. If authorization remains absent, continue only independent authorized work; do not represent publication as complete.

## 7. Proposed architecture and decision status

Read [ARCHITECTURE.md](docs/overhaul/ARCHITECTURE.md) for the full proposal. **It has been presented but not explicitly approved or implemented.** The user has authorized design and wants the overhaul; specific migration/consolidation choices await review.

### Proposed control hierarchy

1. State/observation facade: measured state, validity/freshness, stamped frames, named sensors.
2. Control facade: capability discovery, mode selection, resource ownership, direct joint/gripper commands.
3. Trajectory facade: timed execution, feedback, cancellation, preemption; no mandatory MoveIt.
4. Cartesian facade: IK/targets/streaming; selected planning or Servo implementation is optional.
5. Planning facade: scene-aware planning with one selected scene authority.
6. Task facade: compose manipulation operations and recovery.
7. Policy/agent/teleoperation runners: versioned action specification and observations, using the same command owner and recording semantics.

Proposed design principles:

- Small typed Python core without mandatory ROS/MoveIt/Isaac/camera imports; adapters implement robot and transport details. Start with exercised interfaces rather than a speculative universal framework.
- One writer per resource. Explicit handover between trajectory, streaming, teleoperation, policy, and exclusive SDK mode; reject conflicting commands and unsupported modes.
- Capability-based robot instances, semantic resource/joint names, SI units, timestamps/time domains, freshness, structured results, goal/sequence identity, and measured execution feedback.
- Standard Piper adapter retains official SDK authority for firmware/CAN/torque behavior; generic code must not duplicate that logic.
- Prefer one move_group planning owner with clients for the ROS default. Embedded MoveItPy is an explicit optional profile with its own scene responsibility.
- Evaluate maintained topic-based measured-state ros2_control before replacing vendor transport with a C++ port.
- Pin official description plus small documented overlays for tools/mounts/calibration and qualified model corrections. Generate/qualify URDF, MJCF, USD reproducibly with provenance; validate semantic/FK equivalence separately from dynamics.
- One aperture `w` in metres for the normal gripper; native mappings and mimic jaws behind adapters; confirm actual effective travel.
- Named, instance-scoped frames/resources for other platforms, including later Go2 mounting. Do not force locomotion into an arm-only interface.
- Camera adapters normalize vendor/sim/replay streams and calibration; use observation timestamps. Implement the actual mounted camera before expanding vendors.
- Record commanded actions, measured state, outcomes, synchronization, model/firmware/controller/calibration and policy provenance. Optional versioned LeRobot exporters/policy runners, not mandatory framework forks.
- Keep direct Isaac Lab batched learning separate from the Isaac ROS control profile. Preserve common semantics and state/action definitions across them.
- Prefer tightly coupled workspace-owned code in the superrepo and pinned external dependencies, but preserve histories/licences and agree the migration method before moving submodules. Choose one source-version authority (submodules or vcstool lock), not conflicting locks.
- Recreate relocated environments/builds in an isolated checkout; preserve the existing environment. Keep optional simulator/learning dependencies separate. No edits to the shared Isaac environment.

Hardware fault/watchdog behavior requires qualified semantics. Blindly disabling a gravity-loaded arm can drop it. Python/DDS are not automatically hard real time. Software safeguards are not a safety certification. These are implementation requirements, not evidence of a currently implemented controller.

### Approved actions versus open decisions

Completed/authorized: audit/research, local preservation commits, separate planning branches/docs, backup, source-retrieval corrections, corrected AGENTS path examples, this takeover document.

Still open: exact publication destinations; approval of the proposed implementation architecture; source-management/consolidation method; default planning owner; whether position qualification precedes impedance research (recommended); actual camera/firmware/gripper/mount facts. No shared/system installs or robot motion are authorized by these proposals.

## 8. What remains and how to proceed

Full phases and acceptance criteria: [ROADMAP.md](docs/overhaul/ROADMAP.md).

| Phase | Remaining work | Gate / evidence |
|---|---|---|
| 0 | Publish preservation refs once allowed, verify source reachability and fresh checkout | Exact destinations authorized; child/parent remote SHAs verified |
| 1 | Reconcile saved local patches with official candidates; classify upstreamed/needed/obsolete/experimental work; select compatible pins | Reviewed patch/migration matrix; no blind merge |
| 2 | Isolated environment and official build; canonical description and provenance report | Correct paths/ABI; expanded model, frames, limits and aperture checks |
| 3 | Core/ROS control boundary and mock profile; fix results/time semantics; optional planning | Failure/cancel/preemption/staleness/ownership tests; direct profile independent of MoveIt |
| 4 | MuJoCo adapter/model qualification | Common semantics, headless scenarios, explicit dynamics assumptions |
| 5 | Gazebo qualification using relevant saved fixes | Current description/controller stack; matching functional scenarios |
| 6 | Isaac native ROS profile and separate Lab learning profile | GPU coordination; no shared-env install; Piper-specific integration evidence |
| 7 | Actual camera, calibration, detection-to-grasp task | User camera/mount inputs; sim/replay timestamped workflow |
| 8 | Recording, teleoperation, action chunks, policy/agent integration | Replay/provenance and controller-handover evidence |
| 9 | Supervised hardware qualification, later Go2 composition | Exact hardware approval/presence; firmware-specific qualification |

The next proposed implementation job is the **saved-local-patch versus official-upstream migration matrix**, estimated 30–45 minutes excluding downloads/builds. Inspect refs without moving this checkout; use an isolated integration worktree/check-out when implementing. Proposed branch name: `refactor/official-stack-20261008` (not created).

For each local patch, identify source commit, affected contract, current upstream equivalent, firmware implications, retain/drop/adapt recommendation, and required test. In particular, inspect saved ROS/Gazebo tips and the restored model xacros before recreating their features. Verify current official heads anew; a future head is not silently substituted into the recorded compatibility candidate set.

After agreement, update vendor sources together on integration branches, retain only justified patches, recreate the environment, and validate narrowly. Preserve legacy workflows/history until replacements and recovery are reviewed. Introduce the hierarchy through incremental working profiles rather than a single large untested rewrite.

### Missing user information

- Exact GitHub destination approval: required to finish publishing, pending.
- Which workflows last worked reliably and whether another agent is editing Piper: asked earlier, unanswered; recheck actual Git/status boards.
- Camera model/serial and mounting/TCP measurements: needed later, not a blocker for the patch matrix.
- Arm firmware, actual gripper travel, real limits and actuator behavior: unknown. Establish through manufacturer information and reviewed supervised read-only/hardware steps; SDK/driver constructors may auto-enable hardware, so inspect their paths first.
- Human decision on package consolidation and planning authority: proposal awaiting review.

## 9. Reading and startup procedure for the next agent

1. Read original `HANDOFF.md` first, then current `AGENTS.md`, README, PLAN, TODO, ORCHESTRATION, and the machine README/CLAUDE guides. Those were the user's requested startup sources. Read this takeover file and the overhaul proposal next; original mission order and outdated drift/model assertions are superseded by the conversation recorded here.
2. Read comms INBOX/STATUS/NEEDS_USER/FOR_OTHERS and other workstream status boards. The chat question may not survive agent replacement; the durable pending request is in NEEDS_USER.md.
3. Read PRESERVATION, AUDIT, ARCHITECTURE, ROADMAP, UPSTREAMS, and the two JSON records. PLAN/ImplementationPlan/package graph are historical proposals, not implemented architecture.
4. Inspect actual root and nested Git statuses/remotes/refs. Expected: planning branch, clean sources, sixteen local package archive refs, no publication. If new edits appear, preserve and attribute them before acting. Check `/tmp` backup still exists; do not assume it survives reboot/cleanup.
5. Report the actual state briefly. Resolve the pending push through exact authorization without bypassing review. Do not re-ask for already granted local preservation permission.
6. Obtain user direction on the concrete design decisions and start one bounded job. Do not begin full migration, consolidation, simulation, or hardware merely because a roadmap exists.

Read-only Git inventory commands, from the workspace root:

```bash
git --no-optional-locks status --short --branch
git --no-optional-locks log -6 --oneline
git --no-optional-locks submodule status --recursive
git --no-optional-locks -C src/agx_arm_ros branch -vv
git --no-optional-locks -C src/agx_arm_gzsim branch -vv
```

Do not use `git submodule update --remote`, reset, recursive branch checkout, or source synchronization to make the history look simpler before understanding the saved states.

### Suggested prompt for launching the next agent

> You are taking over Piper Studio on atlas as atlasdev, cwd `/home/atlasdev/projects/ros/piper_studio`. Follow the startup reading order in TAKEOVER.md section 9 and read TAKEOVER.md completely. It contains the latest conversation, preservation state, proposed design, and unfinished work. Legacy implementation and planning are committed on separate branches; no push has executed because exact destination approval is pending after automatic review rejection. Do not bypass that rejection. No runtime or hardware qualification has been performed. Preserve new edits, use comms files and this chat, and report one bounded step at a time. Summarize the actual checkout and unresolved decisions, then continue the approved scope. Ask separately before hardware movement, firmware changes, installs outside the workspace, or irreversible restructuring.

## 10. File index and final cautions about stale state

- [Original handoff](HANDOFF.md): machine/work agreement and earlier brief; several technical claims corrected here.
- [Current agent rules](AGENTS.md): latest user-provided guide, vendor boundaries, environment and deployment requirements.
- [Overhaul entry point](docs/overhaul/README.md): current proposal and links.
- [Preservation record](docs/overhaul/PRESERVATION.md) and [full ref inventory](docs/overhaul/preservation.json).
- [Audit](docs/overhaul/AUDIT.md), [architecture](docs/overhaul/ARCHITECTURE.md), [roadmap](docs/overhaul/ROADMAP.md).
- [Research sources](docs/overhaul/UPSTREAMS.md) and [official candidate SHAs](docs/overhaul/official-candidates.json).
- [Legacy model recovery](docs/archive/legacy-tcp-description/README.md).
- [Status](comms/STATUS.md), [questions](comms/NEEDS_USER.md), [inbox](comms/INBOX.md), [integration facts](comms/FOR_OTHERS.md).

This document is a timestamped handoff, not an automatic authority over later user instructions or live repository state. Its most important distinctions are: **archived versus working, proposed versus approved, researched versus tested, and committed locally versus published**. Preserve those distinctions in every continuation report.
