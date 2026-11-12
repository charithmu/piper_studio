"""Plan with cuRobo in the Go2 world: the arm (with the ZED X Mini) on the dog, the world = the dog's generated box + the plate (+ payload boxes), nothing else.
    scripts/curobo_env.sh run tools/curobo/plan_on_go2.py [--raise-mm N] [--n 20] [--out FILE]
The robot and the world come from piper_assets (setup go2); the planner is built as tools/curobo/validate.py builds it. Start: the zero pose. Goals: the tool poses of random joint
configurations inside the planning limits that are no closer to the world than the arm's base already is (checked on the CPU with the same spheres), plus the named pose `ready` as the first (the "one planning").
--raise-mm N: what-if, the plate N mm thicker (the arm base N mm higher over the dog): the dog's box and the plate move down in the arm's base frame, the arm is unchanged.
Reports success, solve time, goal error, limits, and whether every waypoint of every trajectory stays clear of the world (spheres vs boxes, CPU, with the model's sphere buffer).
"""
import argparse, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src/piper_assets"))
import numpy as np, torch
import piper_assets as pa

ZERO = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]
READY = [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]      # src/piper_control SRDF `ready` (tests compare piper_core.arm_cli.READY with it)


def world_with_raise(mm):
    w = json.loads(json.dumps(pa.world("go2")))
    d = mm / 1000.0
    for n, b in w.items():
        b["pose"][2] -= d                    # everything is lower relative to the arm base
    if "plate" in w:                         # a thicker plate: its top stays at the arm base plane, its bottom follows the dog
        top = w["plate"]["pose"][2] + w["plate"]["dims"][2] / 2 + d
        w["plate"]["dims"][2] += d
        w["plate"]["pose"][2] = top - w["plate"]["dims"][2] / 2
    return w


def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def clearance(P, r, boxes):
    """smallest distance of spheres (centres P, radii r) to any box; negative = inside."""
    best = np.inf
    for b in boxes.values():
        local = (R(b["pose"][3:]).T @ (P - np.array(b["pose"][:3])).T).T
        half = np.array(b["dims"]) / 2
        outside = np.linalg.norm(np.maximum(np.abs(local) - half, 0.0), axis=1)
        inside = np.all(np.abs(local) < half, axis=1)
        d = np.where(inside, -np.min(half - np.abs(local), axis=1), outside) - r
        best = min(best, float(d.min()))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raise-mm", type=float, default=0.0)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    from curobo._src.types.robot import RobotCfg
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    from curobo.types import JointState
    import yaml
    top = pa.curobo_config(setup="go2")
    k = top["kinematics"]
    buf = k.get("collision_sphere_buffer", 0.0)
    world = world_with_raise(a.raise_mm)
    res = {"world": {n: {"dims": b["dims"], "pose": b["pose"]} for n, b in world.items()}, "raise_mm": a.raise_mm, "sphere_buffer_m": buf, "mount_nominal": pa.mount()["NOMINAL"]}
    mp = MotionPlanner(MotionPlannerCfg.create(robot=RobotCfg.create(top), scene_model={"cuboid": world}, collision_cache={"cuboid": 8, "mesh": 2}, graph_planner_algorithm="rrt"))
    mp.warmup(enable_graph=True, num_warmup_iterations=3)
    names, m = mp.joint_names, len(mp.joint_names)
    lim = mp.kinematics.get_joint_limits()
    lo, hi = lim.position[0].cpu(), lim.position[1].cpu()
    spheres = k["collision_spheres"]

    def spheres_at(q):
        st = mp.compute_kinematics(JointState.from_position(torch.as_tensor(q, dtype=torch.float32, device="cuda").reshape(1, m), joint_names=names))
        s = st.robot_spheres.reshape(-1, 4).cpu().numpy()
        return s[:, :3], s[:, 3] - buf          # cuRobo adds the buffer to every radius: take it off, then ask for the clearance with it kept as the margin

    c0, r0 = spheres_at(ZERO)
    clear0 = clearance(c0, r0, world)
    res["start_clearance_mm"] = round(clear0 * 1000, 2)
    res["start_clearance_ok_with_buffer"] = bool(clearance(c0, r0, world) > buf)
    start = JointState.from_position(torch.tensor([ZERO], device="cuda"), joint_names=names)
    goals = [("ready", READY)]
    g = torch.Generator().manual_seed(0)
    tries = 0
    while len(goals) < a.n + 1 and tries < 2000:
        tries += 1
        q = (lo + (hi - lo) * torch.rand(m, generator=g)).numpy()
        c, r = spheres_at(q)
        if clearance(c, r, world) >= clear0 - 0.0005 and clearance(c, r, world) > buf:      # no sphere closer than the arm base already is (the base always sits near the dog), and clear of the buffer
            goals.append((f"random{len(goals)}", q.tolist()))
    rows = []
    for name, q in goals:
        tp = mp.compute_kinematics(JointState.from_position(torch.as_tensor(q, dtype=torch.float32, device="cuda").reshape(1, m), joint_names=names)).tool_poses
        r = mp.plan_pose(tp.as_goal(), start, max_attempts=4)
        if r is None or not bool(r.success.any()):
            r = mp.plan_pose(tp.as_goal(), start, max_attempts=20)          # a failed goal gets a second, longer try
            gc, gr = spheres_at(q)
            if r is None or not bool(r.success.any()):
                rows.append({"goal": name, "success": False, "goal_config_clearance_mm": round(clearance(gc, gr, world) * 1000, 2), "joints": [round(x, 3) for x in q]})
                continue
        tr = r.get_interpolated_plan()
        P = tr.position.reshape(-1, m).cpu().numpy()
        worst = np.inf
        for q_wp in P[::max(1, len(P) // 40)]:
            c, rr = spheres_at(q_wp)
            worst = min(worst, clearance(c, rr, world))
        endp = mp.compute_kinematics(JointState.from_position(tr.position[..., -1:, :].reshape(1, m), joint_names=names)).tool_poses
        rows.append({"goal": name, "success": True, "solve_ms": round(1e3 * float(r.solve_time), 1), "duration_s": round((P.shape[0] - 1) * float(tr.dt.item()), 2), "waypoints": int(P.shape[0]),
                     "goal_err_mm": round(float((endp.position - tp.position).norm()) * 1000, 2), "min_clearance_mm": round(worst * 1000, 2),
                     "clear_of_world_with_buffer": bool(worst > buf), "pos_viol_rad": float(max((P - hi.numpy()).max(), (lo.numpy() - P).max(), 0))})
    ok = [x for x in rows if x["success"]]
    res["plans"] = rows
    res["summary"] = {"goals": len(rows), "success": len(ok), "clear_of_world": sum(x["clear_of_world_with_buffer"] for x in ok), "mean_solve_ms": round(float(np.mean([x["solve_ms"] for x in ok])), 1) if ok else None,
                      "max_goal_err_mm": max((x["goal_err_mm"] for x in ok), default=None), "max_pos_violation_rad": max((x["pos_viol_rad"] for x in ok), default=None)}
    print(json.dumps({k2: res[k2] for k2 in ("raise_mm", "start_clearance_mm", "start_clearance_ok_with_buffer", "summary")}, indent=1))
    print("ready:", rows[0])
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
