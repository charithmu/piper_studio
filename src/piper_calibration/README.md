# piper_calibration

Calibration of the Piper's wrist camera, as its own package so Studio's robot packages stay slim. Today it holds **one** method: hand-eye calibration with a printed ChArUco board and OpenCV. Further methods (another library, intrinsics, a second independent hand-eye) are new modules beside `handeye`, each producing a result file in the same shape so two methods can be compared.

```
pip install piper_calibration[vision]        # [vision] = OpenCV 4.x contrib (NOT 5.x: no calibrateHandEye) + pillow
piper-calibration-board                      # a printable board (PDF/PNG/JSON); print at 100 %, check the scale bar
piper-calibrate --board BOARD.json --look    # camera only: is the board seen cleanly from here?
piper-calibrate --board BOARD.json --dry     # arm only READ: plan the poses and check the paths
piper-calibrate --board BOARD.json           # THE ARM MOVES; you stand at it with `piper-estop` ready
piper-calibrate --offline SESSION_DIR --board BOARD.json     # re-solve saved captures, no hardware
```

| Module | Does | Needs |
|---|---|---|
| `handeye` | board detection, `cv2.calibrateHandEye` (5 algorithms) + `calibrateRobotWorldHandEye`, board-spread and leave-out quality numbers, verdict | OpenCV |
| `kinematics` | FK of the arm with the camera: `piper_core.Kinematics` on `piper_assets` | numpy |
| `planning` | poses that see the whole board and stay clear of it, eased slow paths | numpy |
| `session` | captures on disk, offline solve, `hand_eye.json` | OpenCV |
| `app` | the command; takes the camera and the arm as arguments (`main(argv, open_camera=, connect_arm=)`) | `piper_core` |
| `zed` | ZED X Mini through pyzed, imported only when the camera is opened | pyzed |
| `board` | the printable board | OpenCV, pillow |

The result is a file; **nothing here applies it**. A person reads the verdict and changes the camera pose in Studio (`docs/CALIBRATION.md`, "Applying a result"). Sessions go to `$PIPER_DATA_DIR/calibration` (else `~/data/calibration`).
Tests (`python -m pytest src/piper_calibration`) run the whole app against a fake arm and a fake camera that render the board from the arm's own joints and a true camera pose; no hardware.
