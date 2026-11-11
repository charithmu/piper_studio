# The ZED X Mini: running on Atlas and on the ZED Box

Status 2026-11-11: **written, not tried** (no ZED Box or camera here). The simulators, the description and the Python client are tested; the real-camera launch is tested only up to the wrapper (a stub, `test_zed_launch.py`). The first real run is rung R3.5 of `CHECKLIST.md`.

## Two machines, one set of names
| | Atlas (workstation, x86_64, Ubuntu 24.04, ROS 2 Jazzy, RTX 4090) | ZED Box (Jetson Orin, aarch64, GMSL2 camera) |
|---|---|---|
| Runs | everything: simulators, MoveIt, control, `piper_py`, the real arm (CAN), Isaac | the ZED wrapper only (`zed_camera.launch.py`); the rest only if the box has ROS 2 Jazzy |
| Camera | simulated ZED X Mini (`camera:=zedxm`, default) | the real one (GMSL2 only works on the box or a capture card) |
| Same | frames, topics, image size, `Piper.image()`, `camera_topics()` | the same |

Downstream code never changes between them. Names (`piper_description.camera_topics("piper_")`, also `Piper.camera_topics()`):
`/piper_zed/zed_node/rgb/color/rect/image` (colour), `.../rgb/color/rect/camera_info` (also valid for the depth), `.../depth/depth_registered` (32FC1, metres), frame `piper_zed_left_camera_frame_optical` (the left lens: z forward, x right, y down).
The ZED publishes colour as `bgra8`, the simulators as `rgb8`; `Piper.image()` returns RGB either way. There are no RealSense-style aliases (0.9.0).

## Frames: nothing to translate
The description's camera frames are Stereolabs' own (`zed-ros2-description`, commit 2246824, `name = piper_zed`): `piper_zed_camera_link` (the mounting hole, child of the bracket), `piper_zed_camera_center`, `piper_zed_left_camera_frame(_optical)`, `piper_zed_right_camera_frame(_optical)`.
**One description only:** `zed_camera.launch.py` starts the wrapper with `publish_urdf:=false publish_tf:=false publish_map_tf:=false` and positional tracking off, so every transform comes from this description's `robot_state_publisher`. The pose of the camera comes from the bracket (`config/camera_mounts/zedxm_bracket_v3.yaml`, nominal until the hand-eye calibration, R3.6).

## On the ZED Box (steps for the user; nothing here was run)
1. Look first: `lsb_release -a`, `ros2 --help`, `ros2 pkg list | grep zed`. Jetson Orin images are often Ubuntu 22.04 (JetPack 6, ROS 2 Humble). The wrapper and this launch work on Humble or Jazzy; the Piper stack (this repo) needs Jazzy, so on a Humble box run **only the driver there** and the stack on Atlas.
2. Driver only (no workspace needed): copy `src/piper_control/launch/zed_camera.launch.py` to the box and run `ros2 launch ./zed_camera.launch.py` (needs `zed_wrapper`, zed-ros2-wrapper 5.x, installed or from Stereolabs' Docker image). Options: `prefix:=piper_` (must equal the stack's prefix), `resolution:=SVGA` (960x600 like the simulators; `HD1200`, `HD1080`), `fps:=30`, `params_file:=` (your own override yaml). Check the parameter names against the wrapper version installed.
3. Network: same `ROS_DOMAIN_ID` on both machines (Piper range 70-161). Atlas' LAN is closed by firewall policy, so use the tailnet. Tailscale carries no multicast, so DDS discovery needs a Fast DDS discovery server (`fastdds discovery -i 0 -l <box tailnet IP> -p 11811` on one machine, `export ROS_DISCOVERY_SERVER=<that IP>:11811` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` on both, `ros2 daemon stop`). This needs no firewall change (`tailscale0` is allowed); any ufw or LAN change goes to the user first. Image streams are large (960x600 bgra8 is 2.3 MB per frame): look at `ros2 topic bw` and, if the link is slow, lower `fps`.
4. On Atlas: `source scripts/session.sh <domain>`, then `ros2 topic hz /piper_zed/zed_node/rgb/color/rect/image`, `python -c "from piper_py import Piper; ..."` / `piper` CLI, `tf2_echo piper_zed_left_camera_frame_optical piper_gripper_base` with the stack running (R3.5).
5. Everything on the box (only with ROS 2 Jazzy there): `git clone` the repo with submodules, `uv venv --system-site-packages`, `uv pip sync env/requirements.txt` (MuJoCo, pyAgxArm and the others have aarch64 wheels, not checked), `python -m colcon build --symlink-install`, `source scripts/env.sh` (`session.sh` only adds the VNC `DISPLAY` and `gl` helper of Atlas and is optional). Isaac Sim stays on Atlas.

## Image numbers the simulators use (wide 2.2 mm lens, assumed; datasheet values)
960x600, 110 deg horizontal, fx = fy = 336.1 px, 30 Hz, depth 0.1 to 8 m (`config/camera_models/zedxm.yaml`). On the real camera use the wrapper's own `camera_info` (the factory calibration); `camera_lens:=narrow` selects 80 deg / 0.15 to 12 m. Set the same resolution on the ZED (the launch's default SVGA = 960x600) for a like-for-like comparison with the simulation (R3.7).
