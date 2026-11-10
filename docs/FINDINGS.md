# Findings and hand-offs from Piper Studio

Notes for other projects that use the published wheels (first reader: the `piper_pick` agent). Newest first. Piper Studio does not edit other projects' files; their requests to us are in their own `docs/FINDINGS.md`.

## 2026-11-10 wheels 0.7.0: real arm, gripper, streaming (0.6.0 was built locally and never released)

**What changed (all four wheels are 0.7.0; `piper_mujoco` and `piper_isaac` need `piper_core>=0.7.0`):**

- `ArmBackend` has three more methods: `gripper_opening() -> float|None` (m), `gripper_force() -> float|None` (N; NaN in the simulators) and `set_gripper(width, effort=None, timeout=None) -> float` (blocks, returns the
  measured width). `ArmState` has `gripper_width` and `gripper_force` (NaN when unknown). `PiperSim` and `PiperIsaacSim` accept `effort` and ignore it; `PiperSim` gained `gripper_opening()`. Existing calls `set_gripper(w)` still work and now return the width.
  A class that implements the old protocol is no longer an `ArmBackend` until it has the three methods.
- `RealArm` (`piper_core.real_arm`, needs `pyAgxArm`): `RealArm.connect("can0")` (fails with a clear message when the CAN bus is down), `start()`, `stop()`, `execute(traj)`, `move_to`, `command`, `hold`, `state()`, `abort()`, `set_gripper`, `gripper_command`.
  - `set_gripper(width, effort)`: guarded (width in [0, 0.07] m, effort in (0, `gripper_max_effort`], default 1.0 N, our cap 3.0 N: raise it with `arm.gripper_max_effort = ...`). It ends when the gripper reached the width, **stopped short of it** or timed out;
    `arm.last_gripper` = `GripperResult(reason, width, force)` with reason `reached | stalled | no_motion | timeout`. **A held object is a `stalled` close at a width above the empty-closed width**; the measured force is **negative while gripping** (-2.05 N at 3 N commanded, 2026-11-10), so use its magnitude.
  - `execute(traj, margin_percent=20, mode="fixed"|"per_point", lookahead=0..10)` returns an `ExecResult` (`ok`, `reason`, `detail`, `message`, logs). Reasons: `done`, `guard:limit|margin|step|velocity` (whole trajectory refused before anything moves), `start_too_far`, `bad_trajectory`, `not_ready` (not started, motors off, or another trajectory running), `fault` (a reported fault, or stale feedback), `guard:rejected`, `stopped`, `timeout`.
  - A guard refusal never moves the arm (tests). A guard rejection, a fault, `abort()` or Ctrl-C **while running** stops and holds the present pose (motors stay on). `abort()` is thread-safe; `stop()` refuses while a trajectory runs, works from any state otherwise, and commands nothing if a fault is reported (the operator e-stops).
  - `hold()` keeps the present pose unchanged and does nothing unless the arm is started.
  - `start()`: clears an e-stop if one is active, preflight, motors on, one checked move to zero; safe to call twice. `stop()`: zero, park (joint 5 = 22.0 deg), then the **ending**: `ending="estop"` (default; electronic e-stop, damped lowering, wait until at rest, reset; the arm stays powered and `start()` works again at once) or `ending="disable"` (a plain disable, a harder drop).
- Streaming 25 ms setpoints (cuRobo `interpolation_dt` 0.025): `execute` sends one `move_j` per point on the trajectory's clock; the speed percent is `fixed` (fastest segment + margin) or `per_point` (per segment); `lookahead` sends n points ahead. **None of these has been measured on the arm yet**; `tools/real_arm/arm_stream_test.py` measures them
  (lag, rms, overshoot, timing, speed vs percent) and `piper_core.stream_metrics.analyze(traj, arm.last_stream)` computes the numbers from any run. Until R1 is done (docs/real_arm/CHECKLIST.md) treat tracking quality as unknown. Measured so far: 100 % = 3.0 rad/s holds at 10 % (16-17.5 deg/s on joint 5).
- Park moved from joint 5 = 16.82 to **22.0 deg**; zero is unchanged (all joints 0, j2/j3 0.01 rad inside their limits).

**Not tested on the real arm:** `RealArm.start/execute/move_to/command/stop` (only the SDK tools and the console have moved the arm), speeds above 10 %, 25 ms streaming, Cartesian paths, the Isaac edits above on the GPU (checked with the fake-server client tests only), the gripper with the arm motors off, slip during motion.

**Gaps simulator vs real gripper (for the sim parameters; the simulators were not changed):** no force read-back and `effort` ignored in both simulators; the real gripper held a 10 mm object at 9.4 mm; empty-closed width, stroke and closing time on the real gripper are still to measure (R2).

**Known risks:** the fake-driver ROS plumbing tests (`test_real_plumbing`) are flaky on their own (they fail on a clean run too; unrelated to `RealArm`); the e-stop has been drilled once from park; the arm settles about 5 deg deeper on joint 5 after an e-stop than after a plain disable (27 vs 22-24 deg), still below the park pose.

**Not done here (requests in `piper_pick/docs/FINDINGS.md`, 2026-11-09/10):** per-backend settle tolerance/cap parameters for `execute`, the Isaac chunk size as a duration, an acceleration check in `check_trajectory`. They need a decision from the owner.

## 2026-11-10 ROS path on the real arm (0.7.0)

`backend:=real` now runs `piper_real_driver` (`piper_control/real_driver.py`), our node on `RealArm`, instead of the vendor `agx_arm_ctrl` (still in `external/`, unmodified, unused for the real arm). Same topics as before (`feedback/joint_states`,
`control/joint_states`, `feedback/arm_status|motor_status|gripper_status`), so `command_guard`, ros2_control and `piper_py` are unchanged for callers. New services: `start`, `stop`, `estop`, `clear_estop` (Trigger); `enable_agx_arm` only disables.
`Piper.start()` / `Piper.stop()` use them, so ROS and `RealArm` begin and end a session the same way (clear an e-stop, motors on, zero; zero, park, e-stop, clear); `piper estop` / `Piper.estop()` is the emergency stop. `command_guard` no longer has `go_zero`.
The gripper takes `max_effort` through the gripper action (default 1.0 N, cap 3.0 N). One program drives the arm at a time: the driver holds the same bus lock as `RealArm`. **Not yet run on the real arm** (docs/real_arm/CHECKLIST.md R3).
