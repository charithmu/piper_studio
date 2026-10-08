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
