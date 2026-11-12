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


## Three setups (0.11)
`piper_assets.SETUPS = ("arm", "arm_cam", "go2")`: the robot only, the robot with the ZED X Mini on its bracket, and the full setup on the Go2 (**the default of the viewers and tools**, `DEFAULT_SETUP = "go2"`).
The older `camera=` arguments keep their meaning when `setup` is not given (a real arm on a table is not on a dog); pass `setup=` to choose.
```python
import piper_assets as pa
pa.manifest()["setups"]                    # which this build has ("go2" needs go2_studio's unitree_ros checkout when the wheel is built)
pa.urdf_path(setup="go2")                  # the whole robot: the arm with its usual names (joint1..6, base_link, tcp_link) + the Go2's links (base, FL_hip, ...), meshes beside it
pa.srdf_path(setup="go2")                  # the arm's SRDF + the plate/base pairs that touch by construction
pa.world("go2")                            # the dog as ONE generated box (tools/go2_cover: the bounding box of the dog at its stand pose) + the plate (+ payload boxes) in the arm's base frame, what cuRobo plans in. frame="base": in the Go2 `base` frame
pa.curobo_config(setup="go2"), pa.curobo_world("go2")   # the arm model (unchanged) + the dog as cuRobo's world_model {"cuboid": {...}}
pa.cover_report()                          # provenance of the box: Go2 description hashes, stand pose, size, that every point of the dog is inside
pa.mount()                                 # plate pose and size, base_in_go2 (4x4), the Go2 stand pose; NOMINAL (estimates, not measured on the dock)
```
The arm is planned alone in every setup; the dog is the world. The mount is an estimate until the plate is measured: re-run `tools/release/build_assets.py` after changing `piper_description/go2.py`.
Go2 meshes are Unitree's (BSD-3, `licenses/unitree_ros-BSD3.txt`).

**Which URDF for what.** `pa.urdf_path(setup="go2")` is the whole robot as one tree (dog root, the arm hanging off the plate): right for TF, kinematics, viewers and visual import. **Do not use it as a physics robot**: the dog's links would be part of the arm's articulation, whose links do not collide with each other
(a simulator would let the arm pass through the dog; a GPU run in Isaac showed exactly that). For physics use the ARM's URDF (`pa.urdf_path(setup="arm_cam")`) and the dog and plate as static bodies: `pa.world("go2")` (the boxes), `pa.mount()["dog_in_arm_base"]` (where the dog's frame is in the arm's base frame) and `pa.mount()["floor_z_in_arm_base"]` (the floor).
That is what `piper_mujoco` and `piper_isaac` do.

