# Calibration of the wrist camera (ZED X Mini)

Code: `src/piper_calibration` (own package and wheel; the robot packages do not import it). Today it holds **one** method, hand-eye calibration with a printed ChArUco board and OpenCV. Other methods come later as modules beside it (see "Adding another method").
Step by step on the box for the agent working with the user: `docs/real_arm/BOX_AGENT_CALIBRATION.md`.

## Why
Every point, the target box, the collision world and the grasp goals are in the base frame through `T_base_cam = FK(q) x T_link6_optical`. `T_link6_optical` is what the description carries (`piper_assets.camera_pose("link6")`); today that is the **first hand-eye result, camera mount v4** (provisional: its own verdict is "not acceptable", 4.1 mm rms, `config/camera_mounts/zedxm_bracket_v4.yaml`). Intrinsics are not calibrated here: the ZED's factory K is used.

## What
`piper-calibrate` runs the whole thing: pyzed for the images, `piper_core.RealArm` for the arm, OpenCV for the solve. Library routines only: `cv2.aruco.CharucoDetector` + `cv2.solvePnP` for the board pose, `cv2.calibrateHandEye` (Tsai, Park, Horaud, Andreff, Daniilidis) for `T_link6_optical`, `cv2.calibrateRobotWorldHandEye` as a cross-check. Forward kinematics is Studio's own `piper_core.Kinematics` on the description (one implementation). Modules and their needs: `src/piper_calibration/README.md`.

**Board.** The printed board is the arucogrid.com ChArUco 6x7, 25 mm squares, 18 mm markers, DICT_6X6_250 (arucogrid.com's own detection snippet builds exactly `cv2.aruco.CharucoBoard((6, 7), 0.025, 0.018, DICT_6X6_250)` with default ids 0-20); capture runs at 1920x1200 (`--resolution`, default HD1200) because these markers are lost beyond 0.5 m at 960x600 (tested on renders). `piper-calibration-board` writes an alternative sheet (A4, 6x8 squares of 30 mm, DICT_5X5_100, 35 inner corners) with its JSON and a self-test. Print at 100 %, glue flat on a rigid matt board, check the 100 mm scale bar and one square with a ruler: if a square is not what the sheet says, regenerate the JSON with the measured size. The board is never measured or placed precisely: it only has to lie still, in front of the arm, in view of the camera at the ready pose.

**Environment.** OpenCV must be 4.x with the contrib aruco API (`pip install piper_calibration[vision]`): 5.0.0 has no `cv2.calibrateHandEye`. On the ZED box: a self-contained uv folder `~/projects/piper-calib` made by `scripts/deploy_calibration.sh` (builds the wheels into it; `--push` copies it, the owner runs that); it does not touch `~/piper` or Studio's venv. Sessions are written to `$PIPER_DATA_DIR/calibration` (else `~/data/calibration`).

## Run (on the box; the user present, e-stop in reach)
1. Stop whatever holds the camera and the CAN bus (on the box: `sudo systemctl stop piper-box`).
2. `piper-calibrate --board <board json> --session NAME --look` checks the printed board with the camera only (no arm). `... --dry` reads the arm where it stands (no motion), looks for the board, plans the poses and checks the paths.
3. `piper-calibrate --board <board json> --session NAME`: asks, then Studio `start()`, one slow move to the ready pose, one frame to find the board, then 14 poses (slow, 0.25 rad/s peak, FK clearance above the board plane) with a frame each, back to ready, Studio `stop()`. `--step` asks before every pose. Ctrl-C holds the arm; an error holds it and moves nothing further.
4. Result: `<root>/NAME/hand_eye.json` (+ `pose-NN.npz/jpg`, `plan.json`). Re-solve any time without hardware: `--offline <dir> --board <json>`.
5. Start whatever you stopped; copy the session folder to Atlas; read the verdict.

## Quality numbers (no ruler)
Board position spread over the captures (rms/max mm, orientation deg), a leave-out check (3 folds), agreement between the five methods and the robot-world cross-check, change from the description's value, and the board spread that value gives. Verdict thresholds (indicative, not validated): 10+ captures, spread <= 3 mm rms and 0.5 deg, methods within 1 deg / 5 mm, held-out <= 4 mm.

## Applying a result (a person, never the program)
1. Read the verdict and the issues. A result that is not acceptable is at best a candidate.
2. Add `config/camera_mounts/zedxm_bracket_vN.yaml` (never overwrite a version) with `camera_in_stand` converted from `T_link6_optical` through the description's own chain, `measured:` filled in (date, method, error), and `zedxm_bracket_vN.hand_eye.json` next to it (see v4 for the form and the caveats).
3. Rebuild the data (`tools/release/build_assets.py`, `build_mujoco.py`, `build_isaac.py`), run `scripts/smoke.sh`, and say in the commit what the numbers were. Downstream projects take it with the next release; plans made with the old pose differ.
4. Judge it on data: replay saved real frames with the old and the new pose, then on the arm with the owner (`docs/real_arm/CHECKLIST.md` R3.6).

## Adding another method (the plan: a second, independent calibration with a respected library)
A new module beside `handeye` (for example a ROS `easy_handeye2`/MoveIt-calibration result imported, or a different board and solver), producing the same result shape: `T_link6_optical` (4x4), `verdict`, `board_in_base` spread, the captures used, the library versions. `piper_calibration.handeye.pose_difference(A, B)` compares two results. Agreement between two independent methods on the same arm is worth more than either verdict alone. Not written yet.

## Tested so far (Atlas, no hardware)
Detector pose against rendered boards (0.2 px reprojection, < 3 mm); hand-eye recovery from 16 rendered views (0.14 deg, 1.5 mm); a wrong kinematic model is flagged; the full app against a fake arm and camera (asks before moving, clearance on every path, session end, true X recovered); the package loads no OpenCV/pyzed/SDK on import. `src/piper_calibration/test`, 17 tests. The first real run (session `calib1`, 2026-11-12, by the earlier code in piper_pick) gave v4. **The moved code has not run on the box, the arm or the real camera.** Open: the uv environment on the box, the arm's own FK accuracy (calibration absorbs part of it into X), depth accuracy at 0.3-0.5 m, a checked board scale.
