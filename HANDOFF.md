# Handoff (2026-11-11, evening): Studio owns the cuRobo model (0.10.0, additive)

**For downstream projects (piper_pick, curobo_piper_study, the bake-off).** The cuRobo sphere model of the Piper (robot only, and with the ZED X Mini on its bracket), its hand-tuned policy and the way to regenerate it live here now
(`docs/CUROBO.md`). Import it from the `piper_assets` wheel: `curobo_config(camera)`, `curobo_model_is_current(camera)`, `curobo_provenance(camera)`, `curobo_planner_defaults()`; keep only planning code on your side.
Regenerating needs the optional environment (`scripts/curobo_env.sh setup`); using the model needs no cuRobo. Viewer: `https://atlas.buri-fence.ts.net:8700/piper-studio/curobo/`.
Wheels 0.10.0 in `dist/`. `curobo_piper_study` already consumes it; for `piper_pick` see `~/projects/core/piper_pick/handoff/STUDIO_CUROBO_MODEL.md` (its `camera=` defaults changed since 0.5.0).

---

# Handoff (2026-11-11, later): ZED names everywhere (0.9.0, breaking)

**For downstream projects.** The RealSense-style camera names are gone. Everything (Gazebo, MuJoCo, Isaac, the real ZED X Mini through the ZED ROS 2 wrapper) now uses the wrapper's own:
- topics (prefix `piper_`): `/piper_zed/zed_node/rgb/color/rect/image`, `/piper_zed/zed_node/rgb/color/rect/camera_info`, `/piper_zed/zed_node/depth/depth_registered` (32FC1 m); frame `piper_zed_left_camera_frame_optical` (left lens, z forward).
  Removed: `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`, frames `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame` (now `zed_camera_center`, `zed_left_camera_frame_optical`).
- Ask the library, do not hard-code: `Piper.camera_topics()`, `piper_description.camera_topics(prefix)`, `piper_assets.camera_topics(prefix)`. `piper_assets.camera()["optical_frame"]` is now `zed_left_camera_frame_optical` (no prefix).
- Real camera: `ros2 launch piper_control zed_camera.launch.py` (wrapper with its TF off); guide for Atlas and the ZED Box: `docs/real_arm/ZED_BOX.md` (written, not tried).
- Wheels 0.9.0 (`dist/`); 0.8.0 is superseded. Nothing else in the API changed.

---

# Handoff (2026-11-11): the wrist camera is now the default (0.8.0, breaking)

**For downstream projects (piper_pick, the cuRobo study, anything using the wheels or the contract).** Every description, URDF, SRDF, simulator model and wheel now carries the Stereolabs **ZED X Mini on the user's own bracket**
unless you ask for the robot only. What changes for you:

- `piper_assets.urdf_path(w)` / `srdf_path()` return the **with-camera** files; `camera=None` is the robot only (a call that already passes `None`, like `piper_assets.urdf_path(.07, None)` in piper_pick's frame-contract code, is unchanged).
- `PiperSim(...)` and `PiperIsaacSim(...)` have the bracket and camera by default (`camera=False`: robot only) and new `camera_image(depth=False)`, `camera_pose()` (optical frame in the base frame), `camera_info()`.
  The camera has collision geometry (the arm is heavier at the wrist: servo damping of joints 4-6 differs; MuJoCo and Isaac agree with each other).
- `piper_assets.camera()` / `camera_pose(frame)`: intrinsics K, size 960x600, 110 deg (wide lens, assumed), depth range 0.1-8 m, optical frame `zed_left_camera_frame_optical` (no prefix, since 0.9.0; the LEFT lens: z forward, x right, y down) and its pose in gripper_base, the TCP and link6.
- ROS: `camera:=none` for the robot only; topic names and frame as in 0.8.0 (changed in 0.9.0, see above); size/field of view from `piper_description/config/camera_models/zedxm.yaml`.
- Removed: the earlier camera, the AgileX stand and the `camera_mount*` arguments.
- The pose of the camera is NOMINAL (the user's CAD, bracket version v3); it has not been on the real arm and is not hand-eye calibrated: do not trust it for real-arm grasping before `docs/real_arm/CHECKLIST.md` R3.

Wheels: `dist/` holds 0.8.0 (piper_core, piper_assets, piper_mujoco, piper_isaac; 0.7.0 is gone from `dist/`). Details: `docs/CLEANUP.md` (2026-11-11), `docs/INTEGRATION.md` section 0, decision D14.

---

# Handoff (2026-11-10)

v0.5.0 includes the existing Isaac stepping/recording optimizations, full GUI entry points and four versioned wheels. Validation and limitations: docs/VERSIONS.md.

All test simulators were stopped; gpu-run reservation released. GUI runs use a local display and private unix sockets, with no new TCP service or gateway route. Registry source lists the interactive GUI and benchmark jobs; no hosting or platform files were changed.

Launch: `scripts/isaac.sh run --gui` (ROS) or, with the four wheels installed, `python -m piper_isaac --gui --scene table`. Ctrl-D closes the interactive client.
