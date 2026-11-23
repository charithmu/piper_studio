# piper-calib on the ZED box (`~/projects/piper-calib`)

Self-contained copy of the hand-eye calibration, made by Studio's `scripts/deploy_calibration.sh`. It does not touch the running box server (`~/piper`, the `piper-box` service) or Studio's venv.

    pyproject.toml            the uv project (Python 3.12, OpenCV 4.x contrib headless, NumPy 2, pyzed 5.5, the Studio wheels, pyAgxArm pinned)
    wheels/                   piper_core, piper_assets, piper_calibration (built from the Studio commit that made this folder)
    board/                    the printed board's JSON (arucogrid_6x7_25mm.json) and charuco_6x8_30mm.{pdf,png,json}
    bin/piper-calibrate       the wrapper: `uv run --project ~/projects/piper-calib piper-calibrate`, user site off
    docs/                     CALIBRATION.md (what and why), AGENT_INSTRUCTIONS.md (step by step for the agent working with the user)

One-time setup (installs packages: the user approves it):

    cd ~/projects/piper-calib && ~/.local/bin/uv lock && ~/.local/bin/uv sync

Acceptance check (must print versions and True True, no traceback):

    PYTHONNOUSERSITE=1 ~/.local/bin/uv run --project ~/projects/piper-calib python -c \
      "import cv2, numpy, pyzed.sl as sl, piper_core, piper_assets, piper_calibration; print(cv2.__version__, numpy.__version__, sl.Camera.get_sdk_version(), hasattr(cv2,'calibrateHandEye'), hasattr(cv2.aruco,'CharucoDetector'))"

Why not `~/projects/vision-env`: it has OpenCV 5.0.0, which has no `cv2.calibrateHandEye`, and no Studio wheels.
