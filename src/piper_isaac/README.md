# piper_isaac

`PiperIsaacSim`: the Piper arm in Isaac Sim, no ROS. It implements `piper_core.backend.ArmBackend` like `piper_mujoco.PiperSim` and the real arm, so a planner
(cuRobo, ...) hands it a `Trajectory` and gets an `ExecResult` back.

```python
from piper_core import Trajectory
from piper_isaac import PiperIsaacSim
with PiperIsaacSim(scene="table", gripper_width=0.07) as sim:   # "empty" | "table"; starts Isaac (about 6 s) and sits at the zero pose
    res = sim.execute(Trajectory(joint_names, t, q, dq=dq))      # or Trajectory.from_pairs(...)
    res.ok, res.reason, res.q_cmd, res.q_meas, res.contacts      # contacts: (t, body_a, body_b, penetration m), arm only
    sim.set_gripper(0.0); sim.object_poses()                     # free objects: mass 0.05 kg, friction 1.0
```

What it does: the client checks the whole trajectory (`piper_core.check_trajectory`: limits, 0.01 rad margin, step, velocity, start within 0.05 rad), refuses or
executes it unchanged, resamples to `ctrl_hz` (cubic Hermite with `dq`), and sends the setpoints in chunks of 25 control steps to `isaac/run_server.py`, which runs
in Isaac Sim's own Python environment through `gpu-run` (one process per instance, a unix socket under a private temp directory, stopped and removed by `close()`).
Between chunks the guard is applied to the first setpoint against the measured state; a rejection holds the pose. Simulation time advances only while a command runs.

Needs at run time: Isaac Sim 6.x (an environment script, `$ISAAC_ENV_SH`, default `~/projects/sim/robosim/env.sh`), `gpu-run`, a GPU with about 4 GB free. The runner and the
USDs (with the ZED X Mini camera, the default, and the robot only: `camera=False`) are inside the wheel (built by `tools/release/build_isaac.py` from `scripts/isaac.sh build-usd`).

**Provisional:** the servo gains are the same stand-ins as MuJoCo's (not the identified firmware loop), so tracking error and lag say nothing about the real arm. Self-contacts are
not reported (self-collision is off in the USD). `margin_percent` is accepted and ignored. The table is a PLACEHOLDER (unmeasured). The real arm has never run `RealArm.execute`.

Rates: the defaults are `ctrl_hz=250, physics_hz=250`: a PhysX step costs about 2 ms whatever its size, so this runs at real time on this machine (500/1000 Hz, `PiperSim`'s rates, runs at 0.3x) with the same accuracy (0.021 rad on the parity plan). The guard's step limit (0.15 rad) is per control step, so it is looser per step at 250 Hz. Pass `ctrl_hz=500, physics_hz=1000` to match `PiperSim`.

Full editor for testing: `python -m piper_isaac --gui --scene table` opens an interactive prompt with `sim`; Ctrl-D stops the server.
From Python, pass `gui=True` to `PiperIsaacSim` (headless remains the default). GUI mode also enables `record_start(path)`.
The editor renders while idle without advancing physics. The launcher forwards `DISPLAY` and `XAUTHORITY`; use a display with NVIDIA graphics support.
Headless recording uses `render=True`: frames render only at the requested video fps and encoding runs on a bounded worker queue.
