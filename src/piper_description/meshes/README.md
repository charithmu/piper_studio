# Meshes

`zedxm.stl`: the Stereolabs ZED X Mini (metres, origin at the threaded screw hole in its bottom, x forward, z up), copied unmodified from
https://github.com/stereolabs/zed-ros2-description (`meshes/zedxm.stl`, commit 2246824, release 0.2.0, 2026-10-07; Apache-2.0, copyright Stereolabs). Used for the visual and the collision, as
Stereolabs does. `zedxm_bracket_v1.stl` (ring-centre frame, first version) and `zedxm_bracket_v2.stl` (gripper-base UCS frame, 2026-11-11), each with `_collision.stl`: the user's own clamp bracket for it (Fusion design), made by `tools/build_zedxm_bracket_mesh.py` from the Fusion STL export (the `Camera Mount.step` of 2026-11-11 is not converted yet).
