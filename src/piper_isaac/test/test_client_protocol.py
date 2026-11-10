"""PiperIsaacSim's client side against a fake in-process server (no Isaac, no GPU, no ROS): the rules, the chunking and the bookkeeping."""

import json
import shutil
import socket
import tempfile
import threading
from pathlib import Path

import numpy as np
import pytest

import piper_assets as pa
from piper_core import ArmBackend, Trajectory
from piper_core.backend import BackendError
from piper_isaac import PiperIsaacSim
from piper_isaac.sim import CHUNK

Z = np.array(pa.zero_pose())
D = np.array([1, 1, -1, 1, 1, 1.0])      # joint 3 moves negative (its range is [-2.97, 0])


class FakeServer(threading.Thread):
    """Speaks isaac/run_server.py's protocol with a first-order servo. `stuck_after` freezes the arm (the guard must notice); `contact_at` emits one contact."""

    def __init__(self, path, stuck_after=None, contact_at=None):
        super().__init__(daemon=True)
        self.q, self.t, self.grip, self.calls, self.targets = Z.copy(), 0.0, 0.07, [], []
        self.stuck_after, self.contact_at, self.steps_run = stuck_after, contact_at, 0
        self.srv = socket.socket(socket.AF_UNIX)
        self.srv.bind(path)
        self.srv.listen(1)

    def run(self):
        conn, _ = self.srv.accept()
        f = conn.makefile("rwb")
        for line in f:
            c = json.loads(line)
            self.calls.append(c["cmd"])
            r = {"ok": True}
            if c["cmd"] == "hello":
                r.update(dof_names=[], ctrl_hz=500.0, physics_hz=1000.0, objects=["cube_red"], contacts=True, prefix="piper_", render=False)
            elif c["cmd"] == "state":
                r.update(t=self.t, q=self.q.tolist(), dq=[0.0] * 6, gripper=self.grip)
            elif c["cmd"] == "reset":
                self.q, self.t = np.array(c["zero"]), 0.0
            elif c["cmd"] in ("run", "settle", "command"):
                rows = c["q"] if c["cmd"] == "run" else ([c["q"]] if c["cmd"] == "command" else None)
                out, cont = [], []
                if c["cmd"] == "settle":
                    goal = np.array(c["goal"])
                    for _ in range(int(c["max_s"] * 500)):
                        if np.max(np.abs(self.q - goal)) < c["tol"]:
                            break
                        self.q = self.q + 0.5 * (goal - self.q)
                        self.t += 0.002
                        out.append(self.q.tolist())
                else:
                    for k, row in enumerate(rows):
                        self.targets.append(row)
                        if not (self.stuck_after is not None and self.steps_run >= self.stuck_after):
                            self.q = self.q + 0.5 * (np.array(row) - self.q)
                        self.t += 0.002
                        self.steps_run += 1
                        out.append(self.q.tolist())
                        if self.contact_at == self.steps_run - 1:
                            cont.append([k, "gripper_link1", "cube_red", 0.002])
                r.update(q_meas=out, contacts=cont)
            elif c["cmd"] == "hold":
                pass
            elif c["cmd"] == "gripper":
                self.grip = c["width"]
                r.update(width=self.grip, contacts=[])
            elif c["cmd"] == "objects":
                r.update(cube_red={"pos": [0.3, 0.06, 0.02], "quat_wxyz": [1, 0, 0, 0]})
            elif c["cmd"] == "quit":
                f.write(b'{"ok": true}\n')
                f.flush()
                return
            f.write((json.dumps(r) + "\n").encode())
            f.flush()


@pytest.fixture
def make():
    made, dirs = [], []

    def _make(**kw):
        d = tempfile.mkdtemp(prefix="pisaactest_")
        dirs.append(d)
        srv = FakeServer(str(Path(d) / "s"), **kw)
        srv.start()
        sim = PiperIsaacSim("table", ctrl_hz=500.0, physics_hz=1000.0, connect=str(Path(d) / "s"))
        made.append(sim)
        return sim, srv

    yield _make
    for s in made:
        s.close()
    for d in dirs:
        shutil.rmtree(d, ignore_errors=True)


def plan(dt=0.024956, dur=2.0, amp=(0.1, 0.0, 0.0, 0.1, 0.0, 0.0)):
    t = np.arange(0.0, dur + 1e-9, dt)
    s = t / t[-1]
    p, v = 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
    d = np.array(amp)
    return Trajectory(pa.JOINT_NAMES, t, Z + p[:, None] * d, v[:, None] * d)


def test_is_a_backend_and_state(make):
    sim, srv = make()
    assert isinstance(sim, ArmBackend) and sim.start() and sim.state().q == pytest.approx(Z)
    assert sim.contacts_supported and set(sim.object_poses()) == {"cube_red"}


def test_constructor_checks_like_pipersim():
    with pytest.raises(ValueError):
        PiperIsaacSim("garden", connect="/nonexistent")
    with pytest.raises(ValueError):
        PiperIsaacSim("table", gripper_width=0.5, connect="/nonexistent")


def test_refusals_send_nothing_and_move_nothing(make):
    sim, srv = make()
    before = list(srv.calls)
    far = plan()
    far.q[:] += 0.3 * D
    cases = {"start_too_far": far,
             "guard:margin": Trajectory(pa.JOINT_NAMES, [0, 1.0], [np.zeros(6), np.zeros(6)]),                      # exact zero: j2 and j3 are ON their limits
             "guard:limit": Trajectory(pa.JOINT_NAMES, [0, 5.0], [Z, Z + [0, 0, 0, 2.0, 0, 0]]),
             "bad_trajectory": Trajectory(["a", "b"], [0, 1.0], [[0, 0], [0, 0]])}
    for reason, tr in cases.items():
        r = sim.execute(tr)
        assert not r.ok and r.reason == reason, (reason, r.reason, r.detail)
    assert not [c for c in srv.calls[len(before):] if c in ("run", "settle", "command")]
    assert sim.state().q == pytest.approx(Z) and srv.t == 0.0


def test_execute_is_chunked_logs_every_step_and_settles(make):
    sim, srv = make()
    tr = plan()
    r = sim.execute(tr)
    n_grid = len(np.arange(0.0, tr.t[-1], 0.002)) + 1
    assert r.ok and r.reason == "done"
    assert srv.calls.count("run") == -(-n_grid // CHUNK) and srv.calls.count("settle") == 1
    assert len(r.t) == len(r.q_cmd) == len(r.q_meas) >= n_grid
    assert r.t[n_grid - 1] == pytest.approx(tr.t[-1]) and np.all(np.diff(r.t) > 0)                    # time continues through the settle steps
    assert r.q_meas[-1] == pytest.approx(tr.q[-1], abs=2e-4)
    assert srv.targets[0] == pytest.approx(tr.q[0]) and np.array(srv.targets[n_grid - 1]) == pytest.approx(tr.q[-1])   # the resampled setpoints went out unchanged


def test_contacts_get_the_time_of_their_step(make):
    sim, srv = make(contact_at=3)
    r = sim.execute(plan())
    assert len(r.contacts) == 1
    t, a, b, d = r.contacts[0]
    assert (a, b, d) == ("gripper_link1", "cube_red", 0.002) and t == pytest.approx(3 * 0.002)


def test_guard_stops_a_stuck_arm_and_holds(make):
    sim, srv = make(stuck_after=CHUNK)                                                                # the arm freezes after the first chunk
    r = sim.execute(plan(amp=(1.2, 0.0, 0.0, 0.0, 0.0, 0.0), dur=2.0))
    assert not r.ok and r.reason == "guard:rejected" and "holding" in r.detail and srv.calls[-1] == "hold"
    assert r.guard_events and r.guard_events[0][1] == "guard:rejected" and len(r.q_meas) < 400


def test_command_set_gripper_and_move_to(make):
    sim, srv = make()
    sim.command(Z + 0.05 * D)
    assert srv.q == pytest.approx(Z + 0.025 * D)
    with pytest.raises(BackendError):
        sim.command(Z + 0.5 * D)                                                                          # 0.5 rad from the measured pose: over max_step
    w = sim.set_gripper(0.03, effort=1.0)                                                             # same shape as RealArm.set_gripper: returns the measured width
    assert srv.grip == 0.03 and w == pytest.approx(0.03) and sim.state().gripper_width == pytest.approx(0.03)
    import math
    assert math.isnan(sim.gripper_force())
    with pytest.raises(BackendError):
        sim.set_gripper(0.2)
    assert sim.move_to(Z + [0.2, 0, 0, 0, 0, 0], duration=1.0).ok and sim.stop().ok


def test_defaults_are_the_real_time_rates():
    import inspect
    p = inspect.signature(PiperIsaacSim.__init__).parameters
    assert p["ctrl_hz"].default == 250.0 and p["physics_hz"].default == 250.0


@pytest.mark.parametrize("gui", [False, True])
def test_launcher_preserves_display_only_for_gui(monkeypatch, tmp_path, gui):
    from piper_isaac import sim as module

    env_sh = tmp_path / "env.sh"
    env_sh.touch()
    monkeypatch.setenv("DISPLAY", ":96")
    monkeypatch.setenv("XAUTHORITY", "/tmp/piper-test-auth")
    launched = []
    monkeypatch.setattr(module.subprocess, "Popen", lambda cmd, **kw: launched.append(cmd))
    monkeypatch.setattr(PiperIsaacSim, "_connect", lambda *a: None)
    sim = PiperIsaacSim.__new__(PiperIsaacSim)
    sim.gui, sim.scene = gui, "table"
    sim.ctrl_hz = sim.physics_hz = 250.0
    sim.gripper_width = 0.07
    try:
        sim._launch(tmp_path / "model.usda", env_sh, gui, 1.0)
        cmd = launched[0]
        assert ("--gui" in cmd) == gui
        assert ("--render" in cmd) == gui
        assert ("DISPLAY=:96" in cmd) == gui
        assert ("XAUTHORITY=/tmp/piper-test-auth" in cmd) == gui
    finally:
        sim._log.close()
        shutil.rmtree(sim._dir)
