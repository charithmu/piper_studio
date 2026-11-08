# FOR_OTHERS: facts other threads need (paths, measured values, decisions). The user or Claude copies them into the other thread's INBOX.

## 2026-10-08 UTC: Piper preservation and architecture proposal
- Standard Piper plus AgileX gripper remains the initial acceptance robot; Piper-L policies/gains are not accepted as its hardware model.
- Work is currently source preservation and design only. No robot/simulator/GPU process or shared-environment modification was started.
- Proposed common interfaces include resource ownership, measured state, gripper aperture in metres, trajectory/direct/Cartesian control, optional MoveIt, recording, and versioned policy action specifications.
- Proposed platform integration uses instance-scoped frames and a documented fixed mounting transform; no final mount, TCP, camera extrinsics, or physical limits have been established yet.
- Latest official SDK/ROS/description candidate revisions and the full proposal are under `docs/overhaul/`; they are research candidates rather than a tested compatibility lock.

## 2026-11-08 UTC: piper_studio names changed (downstream copies of `src/`)
- Joints, links and frames are now prefixed: `piper_joint1..6`, `piper_gripper`, `piper_tcp_link`, `piper_base_link`, `piper_camera_color_optical_frame` (launch/xacro argument `prefix`, `""` restores the official names; `piper_py.Piper(prefix=...)`). Camera topics are unchanged (`/camera/color/image_raw`, ...).
- Tools moved: `tools/record_video.py` is now `tools/capture/record_video.py`, `capture_stills.py` likewise; real-arm scripts are in `tools/real_arm/`.
- The `00_studio_overlay` experiment in ml_tests (a copy of `src/`) needs a refresh before it works with the new names.
