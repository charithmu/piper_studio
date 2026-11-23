# Instructions for the agent on the ZED box: hand-eye calibration of the wrist camera

You are the agent on the ZED box (`zedbox`, Jetson Orin, Ubuntu 24.04, Python 3.12). You work **with the user, who is at the robot**. The code lives in Studio (`src/piper_calibration`) and was tested on Atlas (the big
workstation) against a fake arm and camera; it has **never run on this box, this arm or this camera**. Your job: build its environment, run it step by step with the user, and report faithfully.
Studio keeps this file as `docs/real_arm/BOX_AGENT_CALIBRATION.md`; it is copied to `~/projects/piper-calib/docs/AGENT_INSTRUCTIONS.md`; the folder is self-contained.

## Why we do this
A Piper arm (AgileX, Studio's `piper_core.RealArm`) carries a ZED X Mini on its wrist. A pick-and-place pipeline (SAM 3, GraspGenX, cuRobo, all on Atlas) places every point in the
arm's base frame through `T_base_cam = FK(joints) x T_link6_optical`. `T_link6_optical` (the camera's pose in the link6 frame) is today the provisional v4 value.
The camera pose in Studio is the first hand-eye result (camera mount v4, verdict "not acceptable", 4.1 mm rms): provisional. This calibration measures it again with a printed ChArUco board and OpenCV
(`cv2.calibrateHandEye`). The result, `hand_eye.json`, goes back to Atlas, where the owner reads the verdict and decides whether it becomes the next camera mount version.

## Hard rules
1. **The arm moves only with the user standing at it, the e-stop in reach, and approving.** The app asks before every motion stage. Never run the real (non `--dry`, non `--look`) step on your own, in a loop, or from a background job.
2. **Installing is the user's decision.** `uv lock && uv sync` downloads packages: say what and where, wait for a yes. Install nothing else. No system Python changes, no apt, no changes to `~/piper`, Studio's venv, `piper_core`, or `~/projects/vision-env`.
3. **No `sudo` on your own.** `sudo systemctl stop/start piper-box` is for the user to run (give them the exact command).
4. Do not edit the installed source to make a check pass. If something does not work, report the error text and what you tried. The code is tested on Atlas; a change here that Studio does not have is a fork.
5. Nothing here listens on a port or leaves the box. Do not copy frames anywhere; Atlas pulls the session folder itself.

## What is in `~/projects/piper-calib`
`pyproject.toml` (uv project: Python 3.12, `opencv-contrib-python-headless` 4.x, NumPy 2, pyzed 5.5, the Studio wheels piper_core, piper_assets and piper_calibration from `wheels/`, pyAgxArm at its pinned commit),
`board/*.json` and the printable `charuco_6x8_30mm.{pdf,png}`, `bin/piper-calibrate` (wrapper), `docs/CALIBRATION.md` (what and why), `SHA256SUMS`. (Built and copied by Studio's `scripts/deploy_calibration.sh`.)
OpenCV must be 4.x: `~/projects/vision-env` has OpenCV 5.0.0, which has no `cv2.calibrateHandEye` (checked), so do not use it for this.

## Steps

### 0. Look around (read-only)
`cd ~/projects/piper-calib && sha256sum -c SHA256SUMS --quiet && echo ok`; `ls ~/projects/piper-calib/wheels`; `systemctl status piper-box --no-pager | head -5` (is the box server running? it holds the camera and the CAN bus);
`~/.local/bin/uv --version`. Report anything unexpected (for example the wheels missing from `wheels/`, or uv absent).

### 1. Build the environment (ask the user first)
Tell the user: "`uv lock && uv sync` in `~/projects/piper-calib` will download OpenCV 4.x contrib headless, NumPy 2 and pyzed 5.5 from stereolabs.com, and install the Studio wheels from `wheels/` and pyAgxArm from GitHub at a pinned commit, into `~/projects/piper-calib/.venv`. Nothing outside that folder changes. OK?" On yes:

    cd ~/projects/piper-calib && ~/.local/bin/uv lock && ~/.local/bin/uv sync

Acceptance check (must print versions and `True True`, no traceback):

    PYTHONNOUSERSITE=1 ~/.local/bin/uv run --project ~/projects/piper-calib python -c "import cv2, numpy, pyzed.sl as sl, piper_core, piper_assets; print(cv2.__version__, numpy.__version__, sl.Camera.get_sdk_version(), hasattr(cv2,'calibrateHandEye'), hasattr(cv2.aruco,'CharucoDetector'))"

Likely problems: the `piper-core` / `piper-assets` / `piper-calibration` path sources point at `wheels/*.whl` (check they exist); `uv lock` resolving only for `linux aarch64` (set in `pyproject.toml`); a `pyAgxArm` git failure (network); the pyzed wheel URL (it is the same one `vision-env` uses). Fix by reporting; if a pin must change, tell the user and Studio's agent (via the user), and keep `opencv-contrib-python-headless` below 5.
Then run the offline self check of the app itself: `bin/piper-calibrate --help` (prints the usage text, no hardware).

### 2. The board (the user's part)
**The user printed a different board than the one in `board/charuco_6x8_30mm.*`** (that one had issues): a ChArUco from arucogrid.com, 6 columns x 7 rows, **25 mm squares, 18 mm markers, DICT_6X6_250, first marker id 0**, A4, border 1.
Its description is `board/arucogrid_6x7_25mm.json`; use that file. Ask the user to measure one printed square with a ruler: it must be 25 mm (if not, edit `square_m` and `marker_m` in a COPY of the JSON to the measured values, same ratio, and report it).
It must be flat and rigid (glued on foam board or MDF), matt, lying **face up on the bench in front of the arm**, about 0.30-0.50 m ahead of the base, under even light with no glare, and must not move during the run.
**Its markers are small, so the camera runs at 1920x1200 (`--resolution HD1200`, the default).** At 960x600 a synthetic test of this exact board is read to 0.4 m and lost beyond 0.5 m (the ready pose is about 0.6 m away); at 1920x1200 it is read fully up to 0.65 m. If the SDK rejects `HD1200` on this camera, try `--resolution HD1080` and tell us which resolution worked. The calibrated result does not depend on the capture resolution (it is the camera's pose on the arm); picking keeps using 960x600.
If the board is rather close to 0.6 m and still not found at the ready pose, **raise it** (put it on a rigid box 10-20 cm high): the app measures the board's height itself and keeps the clearances relative to it.

### 3. Free the camera and the bus (the user runs the sudo)
The box server holds the camera and the CAN bus. Ask the user to run `sudo systemctl stop piper-box` (confirm the service name with `systemctl list-units | grep -i piper`).
Remember that it has to be started again at the end.

### 4. Camera only: is the board seen? (no arm)
    ~/projects/piper-calib/bin/piper-calibrate --board ~/projects/piper-calib/board/arucogrid_6x7_25mm.json --session look1 --look
Prints "board found: N corners ... distance D m" (expect 30 of 30 corners) or "NOT found" with a hint: *no marker at all* = not in view, too far, blurred or glare; *N markers but not enough corners* = too far or blurred, **or the printed layout differs from the JSON** (the generator's layout was not verified against OpenCV's: if markers are seen cleanly from close by and corners never appear, tell Studio's agent and send `look/pose-00.jpg`, do not edit the code); saves `~/data/calibration/look1/look/pose-00.jpg` (green dots on detected corners). **The arm must be at the ready pose or wherever the user parked it with the board in view**; this step does not touch the arm.
Not found: look at the jpg (glare, blur, board cut off, too far or too near, print scaled); fix the setup, repeat. Needs 12+ of the 35 corners and under 1 px reprojection.

### 5. Dry run: the arm is only read
    ~/projects/piper-calib/bin/piper-calibrate --board ~/projects/piper-calib/board/arucogrid_6x7_25mm.json --session dry1 --dry
Connects to the arm (a firmware read, no motion, no motors), looks for the board from where the arm stands, plans 14 poses, checks every path for clearance above the board plane and writes `plan.json`. It says "dry run: nothing moved".
If it prints "STOPPED: ... only N candidate poses see the whole board", the board is too far from or too near the arm or off to a side: move it and repeat.
Read the plan out to the user: the number of poses, the lowest clearance. **The user decides whether to go on.**

### 6. The real run (the user at the arm, the e-stop in reach: `piper-estop` from a second terminal, or the arm's own switch)
    ~/projects/piper-calib/bin/piper-calibrate --board ~/projects/piper-calib/board/arucogrid_6x7_25mm.json --session calib1 --step
It asks (Enter = yes, anything else = stop): start (Studio `start()`: motors on, checked move to zero, 10 % speed), the move to the ready pose, the pose sequence, then with `--step` each pose. It moves slowly (0.25 rad/s peak).
Watch with the user. If a pose is refused, a guard message appears, or anything looks wrong, stop (anything but Enter, or Ctrl-C holds the arm) and report. After the poses it returns to ready and ends with Studio's `stop()`
(zero, park, e-stop, clear). On an error it holds the arm where it stands and prints how to end the session (`piper-arm`, `piper-estop`); it never moves on its own after an error.
Ask the user to drop `--step` for later runs only if the first one was uneventful.

### 7. Read the result
It prints a summary and writes `~/data/calibration/calib1/hand_eye.json` (+ `pose-NN.npz/jpg`, `plan.json`). Quote the summary to the user as is:
the change from the CAD value (deg, mm), the board position spread over the captures (rms mm), the leave-out error, the method agreement, and the **VERDICT**.
- **acceptable** (10+ captures, board spread <= 3 mm rms and 0.5 deg, methods within 1 deg / 5 mm, held-out <= 4 mm): done; tell the user.
- **NOT ACCEPTABLE**: do not loosen the thresholds and do not copy the file onward as good. Look at the listed issues and at the `pose-NN.jpg` images (skipped poses, glare, motion blur), fix the cause (board flat? print scaled? light?), run again with a new session name, optionally `--poses 20`.
Re-solving saved captures without hardware: `bin/piper-calibrate --board <json> --offline ~/data/calibration/calib1`.

### 8. Put the box back
Ask the user to run `sudo systemctl start piper-box`; check `systemctl status piper-box --no-pager | head -5`. The session folder stays in `~/data/calibration/`.

## What to report back (to the user, and for Studio's agent)
- The acceptance-check output of step 1 (versions) and anything that failed or needed a workaround (exact error text).
- For each run: the command, whether the arm moved, the printed summary and verdict, the number of poses used and skipped, and anything odd (refusals, slow or jerky moves, the arm drifting after a stop, the camera failing to open once).
- The session folder paths. Atlas pulls them with `rsync zedbox:data/calibration/<session>/ ~/data/calibration/<session>/`; do not push anything to Atlas.

## What this does not do
It does not change any file the pipeline uses by itself. The result is applied by a person in Studio only after the verdict has been read (docs/CALIBRATION.md, "Applying a result"); a verdict that is not acceptable is not applied.
Intrinsics stay the ZED factory values; depth accuracy and the gripper tip offset are not measured here.
