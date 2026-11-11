# The ZED X Mini on the ZED Box (GMSL2): how it fits this description

Status 2026-11-11: **written, not tried** (no ZED Box or camera here). The description is ready for it; the driver side is the user's to bring up (rung R3.5 of `CHECKLIST.md`).

## Frames: nothing to translate
The description's camera frames are Stereolabs' own (`zed-ros2-description`, commit 2246824) with `name = piper_zed`, so the ZED wrapper started with **`camera_name:=piper_zed`** talks about the same frames:
`piper_zed_camera_link` (the camera's mounting hole, child of the bracket), `piper_zed_left_camera_frame` / `_optical`, `piper_zed_right_camera_frame` / `_optical`. The one rename: Stereolabs' `piper_zed_camera_center` is `piper_camera_link`
here. The contract's `piper_camera_color_optical_frame` and `piper_camera_depth_optical_frame` are identity aliases of `piper_zed_left_camera_frame_optical` (the ZED's colour and depth are registered to the left lens).

- **One description only:** run the wrapper with its own TF publishing off (the ZED wrapper's `pos_tracking` / `publish_tf` and `publish_map_tf` parameters, and no `robot_state_publisher` from `zed_description`), so the transforms come from this description's `robot_state_publisher` (MoveIt/`piper_control`). Check the parameter names in the wrapper version on the box.
- The pose of the camera on the arm comes from the bracket version (`config/camera_mounts/zedxm_bracket_v3.yaml`, nominal). After hand-eye calibration (R3.6) it comes from the measured file, on every backend.

## Topics: the contract keeps the RealSense names (decision open)
Downstream code (`piper_py`, `piper_pick`) reads `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/aligned_depth_to_color/image_raw` (frame `piper_camera_color_optical_frame`). The ZED wrapper publishes
`zed/zed_node/rgb/image_rect_color`, `.../rgb/camera_info`, `.../depth/depth_registered` (names depend on the wrapper version and `camera_name`). Two ways, **the user decides**:

1. Keep the contract: run `topic_tools relay` (or a launch `remappings=`) from the ZED topics to the three names above on the ZED Box or here. Nothing downstream changes. Depth units: the ZED publishes 32FC1 in metres, as the contract expects.
2. Change the contract to the ZED names: a breaking change recorded in `docs/INTEGRATION.md` and `docs/CLEANUP.md`; the simulators would publish the ZED names too.

## Image numbers the simulators use (wide 2.2 mm lens, assumed; datasheet values)
960x600, 110 deg horizontal, fx = fy = 336.1 px, 30 Hz, depth 0.1 to 8 m (`config/camera_models/zedxm.yaml`). On the real camera use the wrapper's own `camera_info` (it publishes the factory calibration); `camera_lens:=narrow` selects 80 deg / 0.15 to 12 m.
Set the same resolution on the ZED (SVGA/`HD600` mode) for a like-for-like comparison with the simulation (R3.7).
