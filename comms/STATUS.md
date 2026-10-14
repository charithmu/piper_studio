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

## 2026-10-08 19:50 UTC M4: MuJoCo backend (Claude, branch `rewrite`)
- `backend:=mujoco` generates the MJCF from piper_description at launch (mujoco_ros2_control converter, 2 s, cached by content hash) and runs the shared controllers/MoveIt; the shared test passes on mock, Gazebo and MuJoCo (suite run twice, 14/14).
- Generated model: FK equals the official URDF to 1e-6 m; fingers coupled by equality constraints. Actuator model is provisional (position servos + gravity compensation).
- Found and fixed: piper_py could return joint states hundreds of ms stale (reliable queue shared with action callbacks) -> latest-sample QoS in its own callback group; base-link1 contact locking joint1; underdamped gripper.

## 2026-10-08 22:30 UTC M5: Isaac Sim backend + demo page (Claude, branch `rewrite`)
- `backend:=isaac`: Isaac Sim 6.1 as its own headless process (isaac/, scripts/isaac.sh), ROS 2 topics to ros2_control's JointStateTopicSystem; same controllers/MoveIt; all 8 end-to-end checks pass (opt-in test).
- Scripted 11-step demo passes on mock, Gazebo, MuJoCo, Isaac; end states agree within 0.035 rad / 4.3 mm; videos from Isaac (live render) and MuJoCo (replay).
- Fixed on the way: ros2_control before /clock (gate), finger coupling (per-frame in runner), servo damping (MuJoCo-equivalent gains), recorder starvation (own node, deep queue).
- Coordination: no shared Isaac session used; board lines on robosim/handoff/STATUS.md; ROS domains 70-161 only; GPU back to baseline after every run.
- Exposure: tailnet-only route https://atlas.buri-fence.ts.net:8700/piper-studio/demo/ (static page, local :8780, user service web-piper-studio-demo).

## 2026-10-10 13:10 UTC Servo, cameras in all three simulators, control-level audit (Claude, branch `feature/servo-camera`)
- MoveIt Servo (TCP twist / pose) works on every backend; simulated D435 wrist camera works in Gazebo, MuJoCo (30 Hz) and Isaac, on the RealSense topics and frames, with shared scene objects.
- Gazebo camera was blank because the sensor frame lies inside the D435 housing mesh (fixed: sensor at the lens plane) and the server needs `--headless-rendering` (now `gz_server.sh`, with a watchdog against orphans).
- End-to-end test: Gazebo 10/10, MuJoCo 10/10, mock 9 + skip (the camera check). Isaac: see the status page.
- Control levels of the vendor SDK / ROS node mapped against what we expose (page `control.html`): we use firmware-smoothed `move_j` only; MIT, JS, CPV, firmware Cartesian, limits/payload and `arm_status` are not exposed yet.
- Vendor study copies: 26 AgileX repos in `for_reference/agilex/` (git-ignored, commits in clone.log). Real arm still not driven.

## 2026-10-12 UTC real arm, read-only baseline (Claude, branch `feature/architecture-comms`)
- Merged `feature/servo-camera` to `main` (b5b72da) after fixing an intermittent Pilz LIN planning failure (Cartesian deceleration limit now symmetric, -2.25).
- Real Piper connected via gs_usb adapter (`can0`, 1 Mbit/s). Listen-only capture, then `tools/arm_probe.py` (one firmware query frame, no enable/move): firmware **S-V1.7-3**, hardware H-V1.2-1, produced 2025-08-09, SDK profile `default`. Standby, status normal, no error code, all joints disabled, 24 V, driver 27-29 C, no fault bits.
- Resting pose (deg) [3.06, -1.85, 2.32, -4.69, 16.11, 0.48]; j2 < 0 and j3 > 0 sit outside the nominal ranges, gripper reads -4.27 mm (outside 0-70): possible zero offsets after long idle. Not touched.
- Consequences of S-V1.7-3 (below S-V1.8-5): no seamless mode switching, so avoid move_js/MIT here (leaving them needs reset = arm powers off); `move_home` (0x191) needs >= 1.7-4. The maintainer's advice for old-firmware bugs is to upgrade to S-V1.8-9 via support@agilex.ai.
- Next: user runs `arm_probe.py --limits` (read requests only) to compare stored limits with the official URDF; then enable-without-motion.
- 2026-10-12 limits read from the arm (S-V1.7-3): joints 1-5 angle limits equal the official URDF; joint 6 stores +-180 deg (URDF +-120 deg); speed 2.5 rad/s and acceleration 3.5 rad/s^2 per joint (URDF 5, our MoveIt 5/5: too high for the real arm); flange 0.75 m/s, 0.18 rad/s, 1.0 m/s^2, 0.25 rad/s^2. Not corrupted. Action: real-arm limit profile (<= 2.5 / 3.5).

## 2026-10-12 UTC real arm: correction and record (Claude, branch `feature/architecture-comms`)
- Correction to the entries above: values read from the real arm (limits, zero offsets, ratings) are **provisional**: firmware S-V1.7-3 is old and not yet upgraded or fully tested. Nothing in the simulators, MoveIt, controllers or `command_guard` was changed from them; the "real-arm limit profile" idea is parked until after the upgrade.
- Recorded in `docs/real_arm/REAL_ARM.md` (vendor docs digest, firmware gates, control modes, safety semantics, API catalogue, first baseline, open items) with raw outputs in `docs/real_arm/baseline_S-V1.7-3/`. Tools: `tools/arm_probe.py`, `arm_probe_extra.py` (read-only), `arm_enable_test.py` (enable/disable, no motion).
- Open: gripper status read returned None in one run; source of the 0x159 gripper frame; j2/j3/gripper zero offsets; firmware flash procedure (ask support@agilex.ai in writing); explicit speed_percent for backend:=real.

## 2026-10-14 UTC real arm after the firmware upgrade (Claude, branch `feature/architecture-comms`)
- User upgraded the arm to **S-V1.9-1** and calibrated it. New read-only tool `tools/arm_info.py` (census, firmware, status, joints, drivers, limits, ratings, gripper; saves JSON). First run: arm in slave/feedback role, CAN control, MOVE_J, no errors, all joints disabled, profile v189. Nothing moved.
- Changes vs the old firmware (details in `docs/real_arm/REAL_ARM.md` section 11): speed limit 3.0 rad/s, accel 5.0, joint 6 +-170 deg, crash protection now 0 (off), gripper zero fixed (0.14 mm), bus rates halved for motor/driver frames, new frames 0x1C0-0x1C3 and 0x212 (Revo2 hand IDs?).
- No simulator/MoveIt/guard parameters changed. Next: decide speed/crash-protection settings, then the first supervised move (`tools/arm_move_rest.py`, dry run first).

## 2026-10-14 UTC docs cleanup, arm identity (Claude, branch `feature/architecture-comms`)
- Supersedes the 2026-10-12 real-arm entries above (they describe the old firmware S-V1.7-3 and are kept only as history): `docs/real_arm/REAL_ARM.md` is now arm-independent vendor knowledge; facts about one unit live in `docs/real_arm/arms/<arm-id>.md` (`piper-01` is a placeholder label; template in `_template.md`). The old baseline files were removed from the repo (raw data stays in `~/data/ml/isaac/piper_studio/arm_baseline_20261012_S-V1.7-3/`).
- Tools: `arm_probe.py` and `arm_probe_extra.py` are replaced by `arm_info.py`; shared `arm_common.py` picks the SDK profile from the firmware (`arm_enable_test.py` and `arm_move_rest.py` no longer hard-code the old profile). `arm_info.py --arm-id` stamps the JSON.
- The arm reports no serial number over CAN (firmware block: hardware, software, date, motor ratio, node type/number and an unparsed field `30AH`); identify a unit by a label plus the number printed on its casing. The USB-CAN adapter serial identifies the adapter only.

## 2026-10-14 UTC arm_info summary mode, CAN auto-detect, can_up.sh (Claude, branch `feature/architecture-comms`)
- `tools/arm_info.py`: coloured summary by default, `-r/--raw` for the long form, `-c/--cpv-check` for CPV reads (switches the arm to CPV mode, so opt-in), `-i/--channel` for the CAN interface. The census is now a fixed 1.5 s precheck (no options); a silent arm stops the run before anything is sent.
- CAN interface is auto-detected in all arm tools (`arm_common.resolve_channel`: the only CAN interface, else the only one that is up). `scripts/can_up.sh` brings it up (runs sudo itself, `--dry-run`, `--down`, optional bit rate).
- Tested live read-only (summary and raw); `-c` and `can_up.sh` (sudo) not run. Follow-up: `-c` now restores the motion mode it read at the start (P/J/L/C) and verifies it; untested live.

## 2026-10-14 UTC CAN bus naming, CPV restore fix (Claude, branch `feature/architecture-comms`)
- "channel" renamed to "CAN bus" in our tools, scripts and docs: option `-b/--canbus` (was `-i/--channel`), `arm_common.resolve_canbus`, JSON key `canbus`, `scripts/can_up.sh` wording. The SDK's and python-can's own `channel=` keyword is kept at the two call sites.
- `arm_info.py -c`: if the arm was already in MOVE_CPV (from an earlier `-c` run) the restore sets MOVE_J instead of refusing. The first live `-c` run left the arm in MOVE_CPV; the CPV parameters are recorded in `docs/real_arm/arms/piper-01.md`.

## 2026-10-14 UTC arm_info summary: mode panel (Claude, branch `feature/architecture-comms`)
- The summary now leads with: Role (SLAVE/MASTER, from the bus frames and ctrl_mode 6), Motion mode (MOVE_J/P/L/C normal, CPV warning, MIT red), Motors (enabled count), E-stop (software e-stop from status 0x01), Teach mode, Faults (error code and per-joint flags). A master or silent arm, or no firmware answer, shows what is known and stops. Checked live (slave, MOVE_J, disabled) and with synthetic states (e-stop, CPV, teach playback, master, fault).

## 2026-10-14 UTC arm_info summary redesign (Claude, branch `feature/architecture-comms`)
- Summary layout: header, a row of coloured state pills (SLAVE, MOVE_J, MOTORS OFF, E-STOP OFF, TEACH OFF, NO FAULTS; green ok, yellow caution, red problem), then sections State / Arm / Health / Settings stored in the arm / Notes. Joint positions are drawn as gauges inside their stored limits (red diamond when a joint rests outside), temperatures are coloured. Plain text (no colour) when piped or with --no-color.

## 2026-10-14 UTC zero-pose tool, enable test on S-V1.9-1 (Claude, branch `feature/architecture-comms`)
- User ran `arm_enable_test.py` on S-V1.9-1: all six enabled, drift <= 0.08 deg, currents <= 0.08 A, clean disable (recorded in `docs/real_arm/arms/piper-01.md`).
- New `tools/arm_zero.py` (replaces `arm_move_rest.py`): vendor go-zero = `move_j` of all zeros (PyAgxArm_demo `piper_ctrl_go_zero.py`; `reset()` is only for recovery after an e-stop and does not move the arm). Ours adds a read-only preflight, a verified known state (speed set before the mode frame because the SDK's mode frame carries a default 50 % speed, then CAN control + MOVE_J read back), enable, latching the target at the current pose, one slow move watched at 10 Hz, abort-and-hold on any fault, and a prompt before disabling. Compiles, `--help` and the speed check tested; the dry run could not be repeated because the arm went silent (powered off); not run live.

## 2026-10-14 UTC first live run of arm_zero.py: stopped early; fixed; arm_stop.py (Claude, branch `feature/architecture-comms`)
- First live run configured the arm (CAN control, MOVE_J read back, 10 %) and enabled it, then aborted with "drift 0.78 deg" before the zero move. Cause (mine): the tool switched the SDK joint-limit clamp on, and the arm rests with joint 2 at about -2 deg and joint 3 at about +2 deg, outside their stored ranges, so the "hold the current pose" command was clamped to the range edge and itself moved the arm; the drift check then aborted and the arm stayed enabled, partly moved (only the small j2/j3 correction, not the 22 deg of joint 5).
- Fix: no SDK clamp, no latch step, no rate/limit/timeout/size checks; only real faults (status, error code, driver bits) stop the move; default speed 50 % (1..100). The hold used on abort or Ctrl-C pulls joints resting past a stored limit to the range edge. New `tools/arm_stop.py` releases or stops the arm without prompts (disable, `--estop` slow damped lowering, `--reset`).
- Not yet run again.

## 2026-10-14 UTC first real-arm motion works (Claude, branch `feature/architecture-comms`)
- `tools/arm_zero.py` (fixed version) run by the user: known state configured (CAN control + MOVE_J read back, speed 50 %), enabled, one `move_j` to the zero pose in 0.6 s from a 22 deg start, final error 0.06 deg, no fault, clean disable. First motion on the real arm; SDK path only.
- Next: the same through the ROS stack (`backend:=real`, command_guard, `piper activate`), with an explicit `speed_percent` in the launch and the open parameter decisions in `docs/real_arm/arms/piper-01.md` (MoveIt speed 5 -> 3 rad/s, crash protection level, Revo2 frames).

## 2026-10-14 UTC arm_joints.py live joint readout (Claude, branch `feature/architecture-comms`)
- `tools/arm_joints.py`: passive live table of the six joints and the gripper (degrees, `--rad`), updated in place; joints that moved in the last 0.4 s are highlighted; Enter prints/saves the pose (radians, copy-paste) to `~/data/ml/isaac/piper_studio/arm_poses.log`; `--once` for a snapshot. Sends nothing (reads 0x2A5-0x2A8). Tested live (snapshot) and in a pseudo-terminal (in-place redraw, save, quit). `arm_info.py` stays unchanged.

## 2026-10-15 UTC ROS side for the real arm; zero/rest poses (Claude, branch `feature/real-arm-ros`)
- Merged `feature/architecture-comms` to `main` (0f5a43f). New branch `feature/real-arm-ros`.
- Poses: SRDF `zero` (old `rest`: all joints 0, j2/j3 0.01 rad inside) is the start/end pose of every simulation and the demo; new `rest` = zero with joint 5 = 0.2936 rad (16.82 deg, from the pose the user recorded on the arm). Details and the FK heights in `docs/real_arm/arms/piper-01.md`.
- `command_guard`: arm joints are kept `limit_margin` (0.01 rad) inside the URDF limits; values up to `limit_slack` (0.1 rad) outside are clamped instead of rejected (a Piper rests ~0.04 rad past the j2/j3 limits). Launch: `canbus:=` (was `can_port:=`) and `speed_percent:=` (default 20, was unset). `piper_py`: `set_motors`, `move_rest`; CLI `piper enable | disable [--countdown] | zero | rest`; `fake_driver` has the `enable_agx_arm` service. Pilz LIN uses half the acceleration scaling (it failed now and then).
- Tests: new `test_real_resting.py` (fake driver starting at the resting pose: activation, zero and rest through MoveIt, disable/enable) and `test_named_poses.py` (SRDF, startup positions, guard margin and `tools/arm_zero.py` agree). mock/MuJoCo/Gazebo/Isaac end-to-end pass.
- `tools/arm_zero.py --rest` goes zero then rest. Not run on the real arm yet.
