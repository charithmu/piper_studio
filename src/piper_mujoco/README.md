# piper_mujoco

`PiperSim`: the Piper arm in MuJoCo, no ROS. It implements `piper_core.backend.ArmBackend`, so a planner (cuRobo, ...) hands it a
`Trajectory` and gets an `ExecResult` back, exactly as it would from the real arm (`piper_core.RealArm`).

```python
from piper_core import Trajectory
from piper_mujoco import PiperSim
sim = PiperSim(scene="table", gripper_width=0.07)     # "empty" | "table"; starts at the zero pose
res = sim.execute(Trajectory.from_pairs(pairs))        # or Trajectory(joint_names, t, q, dq=dq) from the planner
res.ok, res.reason, res.q_cmd, res.q_meas, res.contacts
```

What it does: the trajectory is checked whole (`piper_core.check_trajectory`: limits, 0.01 rad margin, step, velocity, start within 0.05 rad) and
refused or executed unchanged. It is resampled to `ctrl_hz` (cubic Hermite with `dq`, else linear), each setpoint passes the guard against the
measured state, then goes to the position actuators. Contacts (geom names, penetration) are logged at every control step.

**The actuator model is provisional.** The joints are critically damped position servos with ideal gravity compensation
(`piper_sim/config/mujoco/inputs.xml`, the same file the ROS MuJoCo backend uses), not the identified firmware loop. Tracking error, lag
and torque from this simulation say nothing about the real arm. The geometry, joint limits, masses (CAD) and collision meshes are the real description's.
`margin_percent` is accepted and ignored (it only sets the real arm's speed headroom). The ROS MuJoCo backend has static scene objects, PiperSim's
`table` scene has them as free bodies (pick and place).
The table is a PLACEHOLDER (unmeasured).
