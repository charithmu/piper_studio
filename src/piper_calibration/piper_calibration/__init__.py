"""Calibration of the Piper's wrist camera. Importing this imports neither OpenCV nor the ZED SDK nor the vendor arm SDK.

Modules (each does one thing; a new calibration method goes beside `handeye`, not into it):
  transforms  4x4 pose helpers (numpy only).
  handeye     ChArUco detection + hand-eye solve (OpenCV), quality numbers and verdict. Pure: arrays in, report out.
  kinematics  forward kinematics of the arm with the camera, from Studio's description (piper_core.Kinematics).
  planning    where to look from: candidate poses that see the whole board and stay clear of it; slow eased paths.
  session     captures on disk, offline solve of a saved session, the result file.
  app         the command `piper-calibrate`: look / dry / run / offline against a camera and an arm you pass in.
  zed         the ZED X Mini through pyzed (the only module that imports it, lazily).
  board       the printable board (`piper-calibration-board`).
The result is a file (`hand_eye.json`); nothing here applies it. Studio's camera pose is changed by a person, in a reviewed commit (docs/CALIBRATION.md).
"""
__version__ = "0.13.0"
