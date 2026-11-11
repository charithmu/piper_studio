"""Validate a Piper cuRobo model: audit, zero pose, planning in a table scene, and every returned trajectory against the limits.
The YAML is loaded as it is (only `urdf_path` is set, to the URDF piper_assets ships for that camera).
    gpu-run <cuRobo env>/bin/python tools/curobo/validate.py <model.yml> <camera: none|zedxm> <out.json>
Expected limits and the zero pose come from piper_assets' limits.json and the model's operating scale, not from constants here.
"""
import itertools, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/piper_assets"))
import piper_assets as pa
import numpy as np, torch, yaml

TABLE_TOP_Z = -0.003          # table top 3 mm below the base plane (base spheres stop 3 mm above the base plane (+2 mm buffer = 1 mm above))
SCENE = {"cuboid": {"table": {"dims": [1.4, 1.4, 0.05], "pose": [0.0, 0.0, TABLE_TOP_Z - 0.025, 1, 0, 0, 0]},
                    "box1": {"dims": [0.08, 0.08, 0.12], "pose": [0.25, 0.22, TABLE_TOP_Z + 0.06, 1, 0, 0, 0]},     # obstacles kept clear of the zero-pose gripper (tool at x~0.2, z~0.23, |y|<0.05)
                    "box2": {"dims": [0.08, 0.20, 0.10], "pose": [0.20, -0.28, TABLE_TOP_Z + 0.05, 1, 0, 0, 0]}}}
ZERO = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]      # Piper Studio zero_pose(): 0.01 rad inside the limits
EXPECT = {"vel": 1.5, "acc": 2.5, "jerk": 12.5}      # effective limits we intend (scale 0.5 of 3.0 / 5.0 / 25.0)


def overlaps(kin, k, q, JointState):
    st = kin.compute_kinematics(JointState.from_position(torch.tensor([q], device="cuda", dtype=torch.float32), joint_names=kin.joint_names))
    sph = st.robot_spheres.reshape(-1, 4).cpu(); names = []; rad = []
    for l in k["collision_link_names"]: names += [l] * len(k["collision_spheres"][l]); rad += [s["radius"] + k.get("collision_sphere_buffer", 0.0) for s in k["collision_spheres"][l]]   # cuRobo adds collision_sphere_buffer to every radius
    assert len(names) == len(sph) and max(abs(float(sph[i, 3]) - rad[i]) for i in range(len(sph))) < 1e-4, "sphere order mismatch"
    ign = {a: set(b) for a, b in k["self_collision_ignore"].items()}; buf = k["self_collision_buffer"]; bad = {}
    for i, j in itertools.combinations(range(len(sph)), 2):
        a, b = names[i], names[j]
        if a == b or b in ign.get(a, ()) or a in ign.get(b, ()): continue
        d = float((sph[i, :3] - sph[j, :3]).norm() - sph[i, 3] - sph[j, 3] - buf.get(a, 0) - buf.get(b, 0))
        if d < 0: bad[f"{a}|{b}"] = min(bad.get(f"{a}|{b}", 9.0), d)       # metres here, mm in the result (the study's version mixed the two units)
    return {k: round(v * 1000, 1) for k, v in bad.items()}


def main(path, camera, out, patch_builder=False, n=100):
    from curobo._src.types.robot import RobotCfg
    from curobo.kinematics import Kinematics
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.types import JointState
    raw = yaml.safe_load(open(path)); top = raw.get("robot_cfg", raw); k = top["kinematics"]; res = {"model": Path(path).name, "camera": camera, "patched_builder_baseline": patch_builder}
    k["urdf_path"] = str(pa.urdf_path(0.07, None if camera == "none" else camera)); L = pa.limits(); sc = k["cspace"]["acceleration_scale"][0]
    EXPECT.update(vel=L.velocity * sc, acc=L.acceleration * sc, jerk=L.jerk * sc)
    if patch_builder:                                  # same decisions as scripts/10, applied in memory to the raw builder YAML
        k["base_link"] = "base_link"; nn = len(k["cspace"]["joint_names"]); c = k["cspace"]
        c["max_acceleration"] = [5.0] * nn; c["max_jerk"] = [25.0] * nn; c["velocity_scale"] = [0.5 ** 0.5] * nn; c["acceleration_scale"] = [0.5] * nn; c["jerk_scale"] = [0.5] * nn
        c["default_joint_position"] = [0.0] * nn; top["load_dynamics"] = False
    # ---- audit: nothing at a builder/parser default
    c = k["cspace"]; flags = []
    if k["base_link"] == "world": flags.append("base_link is 'world' (builder default)")
    if any(x == 10.0 for x in c["max_acceleration"]) or any(x == 500.0 for x in c["max_jerk"]): flags.append("acceleration 10 / jerk 500 (builder defaults)")
    if any(abs(a - b) > 1e-9 for a, b in zip(c["default_joint_position"], ZERO)): flags.append("default_joint_position is not Piper Studio zero_pose()")
    if all(v == 0.0 for v in k["self_collision_buffer"].values()) and k.get("collision_sphere_buffer", 0) == 0: flags.append("all collision buffers are 0")
    kin = Kinematics(RobotCfg.create(top).kinematics)      # own RobotCfg: sharing one with the planner made every plan fail
    rc = RobotCfg.create(top)
    res["audit_flags"] = flags
    scene_cfg = MotionPlannerCfg.create(robot=rc, scene_model=SCENE, collision_cache={"cuboid": 8, "mesh": 2}, graph_planner_algorithm="rrt")
    mp = MotionPlanner(scene_cfg); mp.warmup(enable_graph=True, num_warmup_iterations=3); names = mp.joint_names; lim = mp.kinematics.get_joint_limits(); m = len(names)
    eff = {"vel": float(lim.velocity[1, 0]), "acc": float(lim.acceleration[1, 0]), "jerk": float(lim.jerk[1, 0])}
    res["effective_limits"] = {**eff, "expected": EXPECT, "ok": all(abs(eff[x] - EXPECT[x]) < 1e-3 for x in EXPECT),
                               "pos_lo": lim.position[0].tolist(), "pos_hi": lim.position[1].tolist(), "effort": None if lim.effort is None else lim.effort[1].tolist()}
    res["n_spheres"] = sum(len(v) for v in k["collision_spheres"].values())
    res["self_overlap_at_zero"] = overlaps(kin, k, ZERO, JointState)
    # ---- planning in the table scene from exact zero
    g = torch.Generator().manual_seed(0); lo, hi = lim.position[0].cpu(), lim.position[1].cpu()
    start = JointState.from_position(torch.tensor([ZERO], device="cuda"), joint_names=names)
    rows = []; tried = 0
    while tried < n:
        q = (lo + (hi - lo) * torch.rand(m, generator=g)).to("cuda").unsqueeze(0)
        tp = mp.compute_kinematics(JointState.from_position(q, joint_names=names)).tool_poses
        if float(tp.position[0, 0, 0, 2]) < 0.03: continue            # tool must be above the table
        tried += 1; goal = tp.as_goal(); r = mp.plan_pose(goal, start, max_attempts=4)
        if r is None or not bool(r.success.any()): rows.append({"success": False}); continue
        tr = r.get_interpolated_plan(); P, V, A, J = (getattr(tr, x).reshape(-1, m).cpu().numpy() for x in ("position", "velocity", "acceleration", "jerk")); dt = float(tr.dt.item())
        endp = mp.compute_kinematics(JointState.from_position(tr.position[..., -1:, :].reshape(1, m), joint_names=names)).tool_poses
        perr = float((endp.position - tp.position).norm()) * 1000
        rows.append({"success": True, "solve_ms": 1e3 * float(r.solve_time), "duration_s": (P.shape[0] - 1) * dt, "n_wp": int(P.shape[0]), "dt": dt,
                     "pos_viol_rad": float(max((P - hi.numpy()).max(), (lo.numpy() - P).max(), 0)), "v_max": float(np.abs(V).max()), "a_max": float(np.abs(A).max()), "j_max": float(np.abs(J).max()),
                     "max_step_rad": float(np.abs(np.diff(P, axis=0)).max()), "start_err_rad": float(np.abs(P[0] - np.array(ZERO)).max()), "goal_err_mm": perr,
                     "fd_vel_gap": float(np.abs(np.diff(P, axis=0) / dt - 0.5 * (V[1:] + V[:-1])).max())})
    ok = [r for r in rows if r["success"]]
    S = lambda key, f=np.max: float(f([r[key] for r in ok])) if ok else None
    res["planning"] = {"n": len(rows), "success": len(ok), "mean_solve_ms": S("solve_ms", np.mean), "mean_duration_s": S("duration_s", np.mean), "dt": ok[0]["dt"] if ok else None,
                       "max_v": S("v_max"), "max_a": S("a_max"), "max_j": S("j_max"), "max_pos_violation_rad": S("pos_viol_rad"), "max_step_rad": S("max_step_rad"),
                       "max_start_err_rad": S("start_err_rad"), "max_goal_err_mm": S("goal_err_mm"), "max_fd_vel_gap": S("fd_vel_gap"),
                       "n_over_vel": int(sum(r["v_max"] > EXPECT["vel"] + 1e-3 for r in ok)), "n_over_acc": int(sum(r["a_max"] > EXPECT["acc"] + 1e-2 for r in ok)),
                       "n_over_jerk": int(sum(r["j_max"] > EXPECT["jerk"] + 0.1 for r in ok))}
    json.dump(res, open(out, "w"), indent=2); print("DONE", json.dumps({x: res[x] for x in ("audit_flags", "effective_limits", "self_overlap_at_zero", "planning")}, indent=1))


if __name__ == "__main__":
    a = sys.argv[1:]; main(a[0], a[1], a[2], patch_builder="--patch-builder" in a)
