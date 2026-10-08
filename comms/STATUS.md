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
