# FOR_OTHERS: facts other threads need (paths, measured values, decisions). The user or Claude copies them into the other thread's INBOX.

## 2026-10-08 UTC: Piper preservation and architecture proposal
- Standard Piper plus AgileX gripper remains the initial acceptance robot; Piper-L policies/gains are not accepted as its hardware model.
- Work is currently source preservation and design only. No robot/simulator/GPU process or shared-environment modification was started.
- Proposed common interfaces include resource ownership, measured state, gripper aperture in metres, trajectory/direct/Cartesian control, optional MoveIt, recording, and versioned policy action specifications.
- Proposed platform integration uses instance-scoped frames and a documented fixed mounting transform; no final mount, TCP, camera extrinsics, or physical limits have been established yet.
- Latest official SDK/ROS/description candidate revisions and the full proposal are under `docs/overhaul/`; they are research candidates rather than a tested compatibility lock.
