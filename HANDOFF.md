# Handoff (2026-11-11): the wrist camera is now the default (0.8.0, breaking)

**For downstream projects (piper_pick, the cuRobo study, anything using the wheels or the contract).** Every description, URDF, SRDF, simulator model and wheel now carries the Stereolabs **ZED X Mini on the user's own bracket**
unless you ask for the robot only. What changes for you:

- `piper_assets.urdf_path(w)` / `srdf_path()` return the **with-camera** files; `camera=None` is the robot only (a call that already passes `None`, like `piper_assets.urdf_path(.07, None)` in piper_pick's frame-contract code, is unchanged).
- `PiperSim(...)` and `PiperIsaacSim(...)` have the bracket and camera by default (`camera=False`: robot only) and new `camera_image(depth=False)`, `camera_pose()` (optical frame in the base frame), `camera_info()`.
  The camera has collision geometry (the arm is heavier at the wrist: servo damping of joints 4-6 differs; MuJoCo and Isaac agree with each other).
- `piper_assets.camera()` / `camera_pose(frame)`: intrinsics K, size 960x600, 110 deg (wide lens, assumed), depth range 0.1-8 m, optical frame `camera_color_optical_frame` (no prefix; the LEFT lens: z forward, x right, y down) and its pose in gripper_base, the TCP and link6.
- ROS: `camera:=none` for the robot only; topic names unchanged (`/camera/color/image_raw` ...), frame `piper_camera_color_optical_frame`; size/field of view from `piper_description/config/camera_models/zedxm.yaml`.
- Removed: D435, AgileX stand, `camera_mount*` arguments, `camera:=d435`.
- The pose of the camera is NOMINAL (the user's CAD, bracket version v3); it has not been on the real arm and is not hand-eye calibrated: do not trust it for real-arm grasping before `docs/real_arm/CHECKLIST.md` R3.

Wheels: `dist/` holds 0.8.0 (piper_core, piper_assets, piper_mujoco, piper_isaac; 0.7.0 is gone from `dist/`). Details: `docs/CLEANUP.md` (2026-11-11), `docs/INTEGRATION.md` section 0, decision D14.

---

# Handoff (2026-11-10)

v0.5.0 includes the existing Isaac stepping/recording optimizations, full GUI entry points and four versioned wheels. Validation and limitations: docs/VERSIONS.md.

All test simulators were stopped; gpu-run reservation released. GUI runs use a local display and private unix sockets, with no new TCP service or gateway route. Registry source lists the interactive GUI and benchmark jobs; no hosting or platform files were changed.

Launch: `scripts/isaac.sh run --gui` (ROS) or, with the four wheels installed, `python -m piper_isaac --gui --scene table`. Ctrl-D closes the interactive client.
