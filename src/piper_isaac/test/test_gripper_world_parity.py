"""MuJoCo and Isaac Sim are one robot: the same gripper moves and the same World give the same numbers (GPU, opt-in: PIPER_TEST_ISAAC=1).

    PIPER_TEST_ISAAC=1 gpu-run python -m pytest src/piper_isaac/test/test_gripper_world_parity.py -s

The tolerances are what two different contact solvers can agree on; they are written in the asserts. Numbers printed with -s."""

import os

import numpy as np
import pytest

import piper_assets as pa
from piper_core import Body, World
from piper_core.kinematics import Kinematics
from piper_isaac import PiperIsaacSim
from piper_mujoco import PiperSim

pytestmark = pytest.mark.skipif(os.environ.get("PIPER_TEST_ISAAC") != "1", reason="needs Isaac Sim and the GPU: set PIPER_TEST_ISAAC=1")
Z = np.array(pa.zero_pose())


def jaws_world(width, static=True):
    """A box of `width` across the jaw axis between the fingertips at the zero pose (the tool frame of the description)."""
    from scipy.spatial.transform import Rotation as Rot
    T = Kinematics.from_assets().pose(Z, "tcp_link")
    q = Rot.from_matrix(T[:3, :3]).as_quat()
    return World([Body("box", "box", (0.04, width, 0.04), pos=tuple(T[:3, 3]), quat=(q[3], q[0], q[1], q[2]), static=static, mass=0.05)], ground_z=-1.0)


@pytest.fixture(scope="module")
def rows():
    """Run the scenarios on both simulators once; each entry: scenario -> (opening, force, reason, duration)."""
    out = {}
    for name, make in (("mujoco", lambda w: PiperSim(world=w)), ("isaac", lambda w: PiperIsaacSim(world=w))):
        res = {}
        s = make(World(ground_z=0.0))
        s.set_gripper(0.0)
        res["empty close"] = (s.last_gripper.width, s.last_gripper.force, s.last_gripper.reason, s.last_gripper.duration)
        s.set_gripper(0.07, 3.0)
        res["empty open"] = (s.last_gripper.width, s.last_gripper.force, s.last_gripper.reason, s.last_gripper.duration)
        s.close()
        for effort in (1.0, 2.0, 3.0):
            s = make(jaws_world(0.054))
            s.set_gripper(0.0, effort)
            res[f"box {effort:g} N"] = (s.last_gripper.width, s.last_gripper.force, s.last_gripper.reason, s.last_gripper.duration)       # read where the move ended: stalled and still
            s.close()
        out[name] = res
    for k in out["mujoco"]:
        print(f"{k:12s} mujoco {out['mujoco'][k][0]*1000:6.2f} mm {out['mujoco'][k][1]:6.2f} N {out['mujoco'][k][2]:8s} {out['mujoco'][k][3]:.2f} s | "
              f"isaac {out['isaac'][k][0]*1000:6.2f} mm {out['isaac'][k][1]:6.2f} N {out['isaac'][k][2]:8s} {out['isaac'][k][3]:.2f} s")
    return out


def test_empty_moves_end_reached_in_the_real_time_on_both(rows):
    for sim in rows.values():
        assert sim["empty close"][2] == "reached" and sim["empty close"][0] == pytest.approx(0.0, abs=1e-3)
        assert sim["empty open"][2] == "reached" and sim["empty open"][0] == pytest.approx(0.07, abs=1e-3)
        assert 0.5 < sim["empty close"][3] < 0.8 and 0.5 < sim["empty open"][3] < 0.8        # real: 0.28 s to 1 mm + the 0.3 s hold


def test_a_box_stalls_the_close_at_its_width_on_both(rows):
    for sim in rows.values():
        for k in ("box 1 N", "box 2 N", "box 3 N"):
            assert sim[k][2] == "stalled" and sim[k][0] == pytest.approx(0.054, abs=0.002)
    for k in ("box 1 N", "box 2 N", "box 3 N"):
        assert rows["isaac"][k][0] == pytest.approx(rows["mujoco"][k][0], abs=0.0015)       # the two simulators agree on the held width to 1.5 mm


def test_the_grip_force_follows_the_effort_and_plateaus_on_both(rows):
    for sim in rows.values():
        assert sim["box 1 N"][1] == pytest.approx(-1.0, abs=0.25)
        assert sim["box 2 N"][1] == pytest.approx(-2.0, abs=0.25)
        assert sim["box 3 N"][1] == pytest.approx(-2.09, abs=0.25)          # real: -1.04 / -2.09 / -2.09 N


def test_the_same_world_rests_the_same_way_on_both(tmp_path):
    """A slab, a box, a cylinder, a sphere and a mesh (a tetra-less 6 cm cube file) dropped from 5 cm: both simulators rest them on the slab's top face (z = 0)."""
    import trimesh
    mesh = tmp_path / "cube.obj"
    trimesh.creation.box((0.06, 0.06, 0.06)).export(mesh)
    world = World([Body("slab", "box", (0.8, 0.8, 0.04), pos=(0.4, 0.0, -0.02)),
                   Body("box", "box", (0.04, 0.05, 0.06), pos=(0.30, 0.15, 0.08), static=False, mass=0.1),
                   Body("can", "cylinder", (0.03, 0.10), pos=(0.30, -0.15, 0.10), static=False, mass=0.2),
                   Body("ball", "sphere", (0.025,), pos=(0.45, 0.0, 0.075), static=False, mass=0.1),
                   Body("cube", "mesh", (1.0,), pos=(0.45, 0.2, 0.08), static=False, mass=0.15, mesh=str(mesh))], ground_z=-0.3)
    poses = {}
    for name, make in (("mujoco", PiperSim), ("isaac", PiperIsaacSim)):
        s = make(world=world)
        s.move_to(Z, duration=1.0)                               # about a second of simulation time passes
        s.move_to(Z, duration=1.0)
        poses[name] = {k: v[0] for k, v in s.object_poses().items()}
        s.close()
    expect = {"box": 0.03, "can": 0.05, "ball": 0.025, "cube": 0.03}          # centre heights of resting bodies
    for name in poses:
        for body, z in expect.items():
            assert poses[name][body][2] == pytest.approx(z, abs=0.004), (name, body, poses[name][body])
    for body in expect:
        print(f"{body:5s} mujoco {np.round(poses['mujoco'][body], 4)} isaac {np.round(poses['isaac'][body], 4)}")
        assert np.abs(poses["mujoco"][body] - poses["isaac"][body]).max() < 0.006
