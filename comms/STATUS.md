# STATUS (append-only, newest at the bottom). Format: `## YYYY-MM-DD HH:MM step <n>: <title>` + done / evidence / next / blockers.

## 2026-10-08 15:42 UTC step 1: preservation inventory and verified backup
- User explicitly authorized commits and pushes, with legacy implementation preserved separately from planning and overhaul work.
- Read INBOX.md: no messages to acknowledge. Read Go2 and Isaac status boards; no Piper edits reported there.
- Inventoried 13 Git repositories, their indexes, working changes, remotes, and all branch tips.
- Backup: `/tmp/piper-studio-preservation-20261008T153911Z`; 804 working files verified by SHA256, all 13 history bundles verified, indexes and binary patches retained; total 247,031,510 bytes.
- Found additional local committed work on ROS, Gazebo, detection, camera, graspgen, manipulation, and MuJoCo branches that the current superrepo checkout does not include.
- The proposed GitHub repository `charithmu/agx_arm_wristcam` is unavailable; existing `charithmu/agx_arm_eyes` is accessible. Preserve the package rename using that existing remote.
- Next: archive current implementation and additional branch tips; register MuJoCo; push child repositories before the parent; save existing and new planning documents on a separate branch.
- No builds, dependency installs, simulator runs, hardware commands, or service exposure.

## 2026-10-08 15:50 UTC step 2: local legacy commits and design separation
- Legacy superrepo committed on `archive/legacy-work-20261008`: `f6aa65b`.
- Independent legacy child commits: motion `235fdd6`, detection `20678a3`, wrist camera `3a4d689`; all additional local main/ros2 tips preserved on archive branches.
- Existing plans, package graph, handoff, and communication files committed separately as `2f5f491` on `planning/overhaul-20261008`.
- MuJoCo is a recognized submodule; renamed wrist camera uses the accessible `agx_arm_eyes` remote.
- Discovered orphaned description commit `f564787` referenced by saved ROS work. Preserved exact commit on a local branch, a verified 1,272-byte bundle, and a readable patch under `docs/overhaul/legacy/`.
- Static checks: 25 Python files, five package XML files, launcher shell syntax; limited secret-pattern scan had no matches. Existing overlay has missing-path warnings; runtime remains unverified.
- Automatic approval review rejected the batch push; no push executed. Exact destination question is pending; details in NEEDS_USER.md.
- Next: commit and validate the architecture/roadmap proposal, then publish and verify once exact destinations are authorized.

## 2026-10-08 15:54 UTC step 3: overhaul proposal and recoverability checks
- Proposal written under `docs/overhaul/`: architecture, staged roadmap, audit, official candidate revisions, and preservation records.
- Control hierarchy explicitly supports state/direct commands, trajectory execution, Cartesian servoing, optional planning, tasks, recording, and policies without a mandatory MoveIt dependency.
- Proposed generic contracts are capability-based; one command owner per resource; robot/camera/backend mappings remain in adapters.
- Latest official candidate heads reverified with git ls-remote: SDK `841a625`, ROS `e4ccec1`, description `983788b`; these are not a tested compatibility lock.
- The orphaned description bundle was restored into an isolated repository from the original backup: exact commit `f564787` and both xacro blobs recovered. Committed separately as `fe567e8`.
- Markdown links, JSON records, and all 11 top-level gitlink mappings validated. Root agent environment examples now use the correct relocated path.
- Push remains pending the exact-destination authorization question; no remote mutation has been executed.
- Next implementation job after design review: classify saved local patches against official candidates and produce the migration matrix (30–45 minutes, excluding builds/downloads).

## 2026-10-08 15:54 UTC step 4: local preservation/design handover
- Final source-status check: all 13 repositories clean; existing main remains `1bcd80e`.
- Legacy archive is `f6aa65b`; historical plans `2f5f491`; recovered description `fe567e8`; new proposal `4b342ba`.
- Checked 15 local documentation links, JSON parsing, all 11 superrepo submodule mappings, and exact legacy-description recovery in an isolated repository.
- Full evidence and restore instructions are in `docs/overhaul/PRESERVATION.md`; architecture and bounded jobs are in `ARCHITECTURE.md` and `ROADMAP.md`.
- Local commit work and design proposal are complete. Remote publication is not complete: exact-destination authorization is still pending after automatic approval review rejected the batch.
- No implementation overhaul, vendor source update, dependency install, simulator run, or hardware command has started.

## 2026-10-08 16:05 UTC step 5: self-contained agent takeover document
- User requested a complete takeover file covering the initial instruction, plan, all work, changes/decisions, and remaining tasks.
- Created `TAKEOVER.md` without replacing the original `HANDOFF.md`; linked it from the root and overhaul READMEs.
- Includes original startup wording, later scope/authorization changes, latest AGENTS rules, machine boundaries, audit/research, preserved refs and orphan model, proposed hierarchy, phases/acceptance criteria, pending questions, and startup instructions for a new agent.
- Distinguishes local commits from publication and proposals from implemented/tested behavior. Exact-destination approval remains unanswered; no push retried or executed.
- Checked 29 local links in the takeover file and verified the recorded legacy-bundle checksum. Installed metadata versions were rechecked without importing the SDK or invoking hardware.
- Source repositories were clean at task start. Save this documentation on the current planning branch; no runtime/source migration is part of this task.

## 2026-10-08 16:40 UTC cleanup: legacy work consolidated on main (Claude)
- All local package work merged onto each repo's main/ros2 and pushed to the charithmu forks; remote SHAs verified equal to superrepo pins.
- Superrepo main now pins those tips; historical plans and the ChatGPT review moved to docs/archive/. Archive/planning branches deleted (all merged).
- Tag `legacy-final-20261008` marks the legacy state. Superrepo push pending (project settings deny `git push`; user runs it).
- Next: new monorepo architecture on official upstream (agx_arm_ros, pyAgxArm) after user go-ahead.

## 2026-10-08 18:10 UTC M1: new workspace on the official stack (Claude, branch `rewrite`)
- Official agx_arm_ros e4ccec1 (+agx_arm_urdf 983788b) as an unmodified submodule; pyAgxArm 841a625 in .venv via uv.
- New packages: piper_description (official model + TCP + backend switch), piper_bringup (single launch, shared controllers, MoveIt, command_guard), piper_py (API/CLI).
- Tests pass: description unmodified vs official; mock + MoveIt end to end; real-backend plumbing with fake_driver (no arm).
- Found and fixed: NaN commands from inactive ros2_control would reach agx_arm_ctrl as 0.0 (rest pose) -> command_guard; JTC does not enforce URDF limits -> client checks.
- Facts for others: TCP = gripper_base + 0.138 m (flange + 0.1425 m); top-down reach only at TCP z < ~0.12 m; legacy/Menagerie MJCF kinematics differ by up to 11 mm.
- Next: real arm read-only (user present), then Gazebo/MuJoCo/Isaac.

## 2026-10-08 18:45 UTC M3: Gazebo Harmonic backend (Claude, branch `rewrite`)
- `backend:=gazebo` uses the same controllers.yaml and MoveIt config; the shared integration test passes on mock and Gazebo.
- Found and fixed: the official virtual gripper joint is massless, so Gazebo dropped it; DART cannot drive a joint off a limit it rests on (a closed gripper never reopened). Fixed in a generated physics model (tested), not in the official description.
- Gazebo transport is isolated per ROS_DOMAIN_ID (GZ_PARTITION=piper_d<id>); the server runs in-process (ros_gz_sim gzserver).
- Next: MuJoCo, then Isaac. Real arm still waiting for the user.
