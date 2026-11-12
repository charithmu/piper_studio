"""PiperSim against the backend contract (no ROS, no hardware)."""

import json

import numpy as np
import pytest

import piper_assets as pa
from piper_core import ArmBackend, Trajectory
from piper_core.backend import ARM_JOINTS
from piper_mujoco import PiperSim

Z = np.array(pa.zero_pose())


def plan(dt=0.024956, dur=2.0, amp=(0.6, 0.0, 0.0, 0.4, 0.0, 0.0), start=Z, end_at_zero=False):
    """A smooth rest-to-rest move away from `start` (minimum jerk), with velocities, like a cuRobo plan."""
    t = np.arange(0.0, dur + 1e-9, dt)
    s = t / t[-1]
    p, v = 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
    d = np.array(amp)
    return Trajectory(pa.JOINT_NAMES, t, start + p[:, None] * d, v[:, None] * d)


@pytest.fixture(params=["empty", "table"])
def sim(request):
    s = PiperSim(request.param)
    yield s
    s.close()


def test_is_a_backend_and_starts_at_zero(sim):
    assert isinstance(sim, ArmBackend)
    assert sim.state().q == pytest.approx(Z, abs=1e-9) and sim.start()
    assert sim.limits_info.gripper_stroke == 0.07
    assert sim.object_poses().keys() == ({"cube_red", "cube_green", "cylinder_blue"} if sim.scene == "table" else set())


def test_zero_pose_with_open_gripper_has_no_contact(sim):
    c = []
    for _ in range(200):
        sim._step(c, sim.time)
    assert c == [] and sim.state().q == pytest.approx(Z, abs=1e-6)


def test_out_and_back_ends_at_zero_without_contacts(sim):
    out = plan()
    r = sim.execute(out)
    assert r.ok and r.reason == "done" and r.contacts == []
    assert len(r.t) == len(r.q_cmd) == len(r.q_meas) and np.max(np.abs(sim.state().q - out.q[-1])) < 1e-3
    lo, hi = pa.limits().as_arrays("planning")
    q_now = np.clip(sim.state().q, lo, hi)                                         # a planner clamps its start into the planning limits (the arm settles a hair inside the margin band)
    back = plan(start=q_now, amp=tuple(Z - q_now))                                 # planned from state().q, like a planner in a loop
    back.q[0] = q_now
    r = sim.execute(back)
    assert r.ok and np.max(np.abs(sim.state().q - Z)) < 1e-3                       # within 1 mrad of zero


def test_25ms_and_2ms_plans_of_the_same_move_end_at_the_same_pose():
    ends = []
    for dt in (0.024956, 0.002):
        s = PiperSim()
        r = s.execute(plan(dt=dt))
        assert r.ok
        ends.append(s.state().q)
    assert np.max(np.abs(ends[0] - ends[1])) < 1e-3


def test_refusals_move_nothing(sim):
    before = sim.state().q.copy()
    t0 = sim.time
    bad_limit = plan(); bad_limit.q[5, 3] = 2.0
    exact_zero = plan(start=np.zeros(6))
    far = plan(start=Z + np.array([0.1, 0, 0, 0, 0, 0]))
    jump = Trajectory.from_pairs([(0.0, Z), (0.002, Z + np.array([0.2, 0, 0, 0, 0, 0]))])
    for traj, reason in [(bad_limit, "guard:limit"), (exact_zero, "guard:margin"), (far, "start_too_far"), (jump, "guard:step")]:
        r = sim.execute(traj)
        assert not r.ok and r.reason == reason, (reason, r.reason, r.detail)
    assert sim.time == t0 and sim.state().q == pytest.approx(before)


def test_command_goes_through_the_guard(sim):
    sim.command(Z + np.array([0.01, 0, 0, 0, 0, 0]))
    from piper_core import BackendError
    with pytest.raises(BackendError):
        sim.command([3.0, 0, 0, 0, 0, 0])
    sim.hold()
    sim.reset()
    assert sim.state().q == pytest.approx(Z, abs=1e-9)


def test_move_to_and_stop_return_to_zero(sim):
    r = sim.move_to(Z + np.array([0.5, 0.3, -0.3, 0.2, 0.2, 0.1]))
    assert r.ok and np.max(np.abs(sim.state().q - (Z + [0.5, 0.3, -0.3, 0.2, 0.2, 0.1]))) < 2e-3
    assert not sim.move_to([2.7, 0, 0, 0, 0, 0]).ok                                # outside the limits: refused
    assert sim.stop() and np.max(np.abs(sim.state().q - Z)) < 1e-3


def test_gripper_contract_matches_the_real_arm_api():
    """Same shape as RealArm: set_gripper(width, effort=None, timeout=None) -> measured width, gripper_opening(), gripper_force() (NaN here), width in state()."""
    import math
    s = PiperSim("table", gripper_width=0.07)
    assert s.gripper_opening() == pytest.approx(0.07) and s.state().gripper_width == pytest.approx(0.07) and math.isnan(s.gripper_force())
    w = s.set_gripper(0.03, effort=1.0)
    assert w == pytest.approx(0.03, abs=1e-3) and s.state().gripper_width == pytest.approx(w)
    assert s.set_gripper(0.05, 2.0, 1.5) == pytest.approx(0.05, abs=1e-3)
    with pytest.raises(Exception, match="outside"):
        s.set_gripper(0.08)


def test_gripper_and_table_scene():
    s = PiperSim("table", gripper_width=0.07)
    s.set_gripper(0.03)
    assert s.model.joint("gripper").id >= 0 and s.data.qpos[s.model.joint("gripper").qposadr[0]] == pytest.approx(0.03, abs=1e-3)
    s.reset()
    assert s.data.qpos[s.model.joint("gripper").qposadr[0]] == pytest.approx(0.07)
    with pytest.raises(Exception):
        PiperSim(gripper_width=0.08)


def test_wheel_urdf_and_mjcf_agree_on_the_zero_pose_tcp():
    """The URDF a planner reads and the MuJoCo model place link 6 at the same place at zero (kinematics from one description)."""
    import xml.etree.ElementTree as ET
    sim = PiperSim()
    root = ET.fromstring(pa.urdf_path(0.07).read_text())
    joints = {j.get("name"): j for j in root.findall("joint")}
    p = np.zeros(3)
    # forward kinematics of the URDF chain to gripper_base at zero pose
    R = np.eye(3)
    def rot(rpy):
        r, pp, y = rpy
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(pp), np.sin(pp), np.cos(y), np.sin(y)
        return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
    def axis_rot(ax, a):
        ax = np.array(ax) / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    chain = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    parent = {j.find("child").get("link"): j for j in root.findall("joint")}
    link, path = "link6", []
    while link in parent:
        path.append(parent[link]); link = parent[link].find("parent").get("link")
    for j in reversed(path):
        o = j.find("origin")
        p = p + R @ np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        R = R @ rot([float(v) for v in o.get("rpy", "0 0 0").split()])
        if j.get("name") in chain:
            a = Z[chain.index(j.get("name"))]
            R = R @ axis_rot([float(v) for v in j.find("axis").get("xyz").split()], a)
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    assert sim.data.body("link6").xpos == pytest.approx(p, abs=1e-6)


# ---- the ZED X Mini wrist camera (the default) and the robot-only variant -----------------------------------------------------------------
def _link6_pose(sim):          # the converter fuses the fixed flange, gripper base, bracket and camera into link6
    b = sim.model.body("link6").id
    T = np.eye(4)
    T[:3, :3] = sim.data.xmat[b].reshape(3, 3)
    T[:3, 3] = sim.data.xpos[b]
    return T


def test_camera_is_on_by_default_and_sits_where_the_description_puts_it():
    """The optical frame in the simulator equals link6 x the description's optical pose (piper_assets.camera_pose), at several arm poses."""
    sim = PiperSim(scene="empty")
    assert sim.camera_enabled and sim.model.ncam == 1
    rng = np.random.default_rng(0)
    lo, hi = pa.limits().as_arrays("planning")
    for _ in range(4):
        q = rng.uniform(lo, hi)
        for a, v in zip(sim._qadr, q):
            sim.data.qpos[a] = v
        import mujoco
        mujoco.mj_forward(sim.model, sim.data)
        want = _link6_pose(sim) @ pa.camera_pose("link6")
        got = sim.camera_pose()
        assert np.abs(got[:3, 3] - want[:3, 3]).max() < 1e-4, "optical frame position differs from the description"
        assert np.abs(got[:3, :3] - want[:3, :3]).max() < 1e-4, "optical frame orientation differs from the description"


def test_camera_image_has_the_model_size_and_valid_depth():
    pytest.importorskip("mujoco")
    sim = PiperSim(scene="table")
    try:
        rgb, depth = sim.camera_image(depth=True)
    except Exception as e:      # no OpenGL backend on this machine
        pytest.skip(f"no offscreen OpenGL: {e}")
    c = sim.camera_info()
    assert rgb.shape == (c["height"], c["width"], 3) and depth.shape == (c["height"], c["width"])
    assert np.isfinite(depth).mean() > 0.2 and np.nanmin(depth) >= c["depth_min"] and np.nanmax(depth) <= c["depth_max"]
    assert abs(c["K"][0][0] - (c["width"] / 2) / np.tan(np.radians(c["hfov_deg"]) / 2)) < 1e-6
    # MuJoCo's vertical field of view is the pinhole one of the horizontal field of view
    vfov = np.degrees(2 * np.arctan(np.tan(np.radians(c["hfov_deg"]) / 2) * c["height"] / c["width"]))
    assert abs(sim.model.cam_fovy[0] - vfov) < 0.01
    sim.close()


def test_robot_only_variant_has_no_camera_and_no_bracket():
    sim = PiperSim(scene="empty", camera=False)
    assert not sim.camera_enabled and sim.model.ncam == 0
    with pytest.raises(RuntimeError):
        sim.camera_image()
    with_cam = PiperSim(scene="empty")
    assert with_cam.model.ngeom > sim.model.ngeom                      # bracket and camera collision shapes are in the default model
    assert with_cam.start().ok and sim.start().ok


# ---- the full setup on the Go2 (nominal mount) -----------------------------------------------------------------------------------------------------------

def _arm_vs_dog_contacts(sim):
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    m = sim.model
    return sum(1 for c in sim.data.contact[:sim.data.ncon] if sim._arm_body[m.geom_bodyid[c.geom1]] != sim._arm_body[m.geom_bodyid[c.geom2]])


@pytest.fixture(scope="module")
def go2_sim():
    from piper_mujoco.sim import data_dir
    if "go2" not in json.loads((data_dir() / "manifest.json").read_text()).get("setups", []):
        pytest.skip("this build has no go2 setup")
    return PiperSim(setup="go2")


def test_go2_setup_has_the_same_arm_and_the_dog_is_one_fused_posed_body(go2_sim):
    import mujoco
    m = go2_sim.model
    assert go2_sim.setup == "go2" and go2_sim.camera_enabled and m.nu == 7 and [m.joint(j).name for j in ARM_JOINTS]
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)]
    assert names[:2] == ["world", "link1"] and "camera" in [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)]
    # the legs are frozen: the only joints are the arm's and the gripper's; what collides with the arm is the planner's world: the dog's ONE generated box and the plate, no mesh of the dog
    assert m.njnt == 9
    colliding = [g for g in range(m.ngeom) if m.geom_bodyid[g] == 0 and m.geom_contype[g] and m.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
    meshes = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) for g in colliding if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    assert meshes == ["base_link"]                                                  # the only colliding mesh in the fused body is the arm's own base, none of the dog's
    boxes = [g for g in colliding if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX]
    sizes = sorted(tuple(np.round(m.geom_size[g] * 2, 4)) for g in boxes)
    # the arm base is the world origin (as in every other model); the floor is where the dog's feet are, the dog's box the planner's world box
    mujoco.mj_forward(m, go2_sim.data)
    assert go2_sim.data.xpos[m.body("link1").id][2] == pytest.approx(0.123, abs=0.01)
    floor = next(g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE)
    assert m.geom_pos[floor][2] == pytest.approx(-0.4282, abs=1e-3)
    import piper_assets
    dog = piper_assets.world("go2")["dog"]
    assert m.geom_pos[next(g for g in boxes if abs(m.geom_size[g][0] - 0.35313) < 1e-3)] == pytest.approx(dog["pose"][:3], abs=1e-4)
    assert sizes == [(0.14, 0.09, 0.036), (0.7063, 0.328, 0.4232)]


def test_go2_named_poses_are_clear_of_the_dog_and_a_pose_through_the_dog_is_not(go2_sim):
    go2_sim.reset()
    assert _arm_vs_dog_contacts(go2_sim) == 0                                    # zero
    for a, v in zip(go2_sim._qadr, [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]):             # ready
        go2_sim.data.qpos[a] = v
    assert _arm_vs_dog_contacts(go2_sim) == 0
    for a, v in zip(go2_sim._qadr, [-1.555, 3.05, -0.579, -0.613, 1.073, -0.134]):   # the deepest of a random sweep (178 mm in the sphere model): the arm folded back into the dog
        go2_sim.data.qpos[a] = v
    assert _arm_vs_dog_contacts(go2_sim) > 0
    go2_sim.reset()


def test_go2_the_dogs_box_blocks_the_arm_and_contacts_are_reported(go2_sim):
    """Same motion as the Isaac test (test_piper_isaac.py): the arm folded back into the dog is stopped by the dog's box, the contacts are with `dog_cover`."""
    from piper_core import Trajectory
    go2_sim.reset()
    goal = np.array([-1.555, 3.05, -0.579, -0.613, 1.073, -0.134])
    n = int(5.0 / 0.025) + 1
    t = np.linspace(0, 5.0, n)
    sm = (10 * (t / 5) ** 3 - 15 * (t / 5) ** 4 + 6 * (t / 5) ** 5)[:, None]
    res = go2_sim.execute(Trajectory(pa.JOINT_NAMES, t, Z + sm * (goal - Z), np.zeros((n, 6))))
    q = np.array(go2_sim.state().q)
    assert np.abs(q - goal).max() > 0.5, "the arm went through the dog"
    assert any("dog_cover" in (c[1], c[2]) for c in res.contacts)
    go2_sim.reset()


def test_go2_setup_arguments():
    from piper_mujoco.sim import data_dir
    with pytest.raises(ValueError):
        PiperSim(setup="dog")
    if "go2" in json.loads((data_dir() / "manifest.json").read_text()).get("setups", []):
        with pytest.raises(ValueError):
            PiperSim(setup="go2", scene="table")
    assert PiperSim(setup="arm").camera_enabled is False and PiperSim(setup="arm_cam").camera_enabled is True
