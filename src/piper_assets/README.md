# piper_assets

The Piper arm as data, for projects that do not run ROS (planners such as cuRobo, simulators, viewers). Generated at release time by
`tools/release/build_assets.py` from the Piper Studio description, so every consumer gets the same model; nothing here needs ROS to read.

```python
import piper_assets as pa
pa.urdf_path(gripper_width=0.07)                # flat URDF, no prefix (joint1..joint6, base_link, tcp_link), meshes by relative path, WITH the ZED X Mini wrist camera on its bracket (camera=None: the robot only)
pa.camera()                                      # the simulated wrist camera: K, size, field of view, depth range, optical frame and its pose in gripper_base / tcp_link / link6
pa.srdf_path()                                   # self-collision pairs (Adjacent / Never) for the same links
lim = pa.limits()                                # lim.hard["joint2"] -> (lo, hi); lim.planning = hard shrunk by the 0.01 rad margin; lim.sources
pa.zero_pose()                                   # the one start/end pose: [0, 0.01, -0.01, 0, 0, 0]
pa.scene()                                       # shared objects and the PLACEHOLDER table (unmeasured)
pa.manifest()                                    # version, source commit, file hashes
```

Plan inside `lim.planning`, start and end at `zero_pose()`: a trajectory then passes `piper_core.backend.check_trajectory` and is executed
unchanged. `urdf_path(width)` for a width other than 0.0 or 0.07 writes a URDF into the user cache (mesh paths made absolute).
Licences of the third-party meshes: `data/licenses/`.
