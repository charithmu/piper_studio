#!/usr/bin/env python3
"""Build the static demo page from recorded demo runs (scripts/run_demo.sh).

    python tools/build_demo_page.py DEMO_DIR SITE_DIR

DEMO_DIR holds mock|gazebo|mujoco|isaac .json (piper_py demo records) and optional isaac.mp4 / mujoco.mp4.
Needs the workspace env (xacro, matplotlib) and a built workspace (for the TCP forward kinematics).
"""

from __future__ import annotations

import datetime
import html
import json
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import xacro  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/piper_description/tools"))
import model_audit  # noqa: E402
import site_pages  # noqa: E402

BACKENDS = ["mock", "gazebo", "mujoco", "isaac"]
LABEL = {"mock": "Mock (ideal)", "gazebo": "Gazebo Harmonic", "mujoco": "MuJoCo 3.12", "isaac": "Isaac Sim 6.1"}
COLOR = {"mock": "#6b7280", "gazebo": "#d97706", "mujoco": "#2563eb", "isaac": "#16a34a"}
ARM = [f"joint{i}" for i in range(1, 7)]


def load_runs(demo: Path) -> dict:
    runs = {}
    for b in BACKENDS:
        f = demo / f"{b}.json"
        if f.exists():
            r = json.loads(f.read_text())
            t0 = r["steps"][0]["t_start"]
            r["t"] = np.array(r["t"]) - t0
            for s in r["steps"]:
                s["t_start"] -= t0
                s["t_end"] -= t0
            r["q"] = np.array([[np.nan if v is None else v for v in row] for row in r["q"]])
            runs[b] = r
    return runs


def tcp_model():
    urdf = xacro.process_file(str(Path(get_package_share_directory("piper_description")) / "urdf/piper.urdf.xacro")).toxml()
    tmp = Path("/tmp/piper_demo_tcp.urdf")
    tmp.write_text(urdf)
    return model_audit.load_urdf("tcp", tmp)


def tcp_path(model, q):
    out = np.zeros((len(q), 3))
    for i, row in enumerate(q):
        out[i] = model.fk(dict(zip(ARM, row[:6])), "tcp_link")[:3, 3]
    return out


def at(run, t):
    i = min(np.searchsorted(run["t"], t), len(run["t"]) - 1)
    return run["q"][i]


def main(demo: Path, site: Path):
    runs = load_runs(demo)
    site.mkdir(parents=True, exist_ok=True)
    model = tcp_model()
    for r in runs.values():
        r["tcp"] = tcp_path(model, r["q"])

    # ---- plots -------------------------------------------------------------------------------------------
    plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.25, "axes.spines.top": False,
                         "axes.spines.right": False})
    fig, axes = plt.subplots(4, 2, figsize=(11, 9), sharex=True)
    names = ARM + ["gripper"]
    for ax, j in zip(axes.flat, range(7)):
        for b, r in runs.items():
            ax.plot(r["t"], r["q"][:, j], color=COLOR[b], lw=1.3, label=LABEL[b], alpha=0.9)
        ax.set_ylabel(f"{names[j]} ({'m' if j == 6 else 'rad'})")
    axes.flat[7].axis("off")
    axes.flat[7].legend(*axes.flat[0].get_legend_handles_labels(), loc="center", frameon=False)
    for ax in axes[-1]:
        ax.set_xlabel("time since first step (s)")
    fig.suptitle("Joint trajectories of the same scripted sequence", y=0.995)
    fig.tight_layout()
    fig.savefig(site / "joints.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.2, 4.6))
    for b, r in runs.items():
        ax.plot(r["tcp"][:, 0], r["tcp"][:, 2], color=COLOR[b], lw=1.5, label=LABEL[b], alpha=0.85)
    ax.set_xlabel("TCP x (m)")
    ax.set_ylabel("TCP z (m)")
    ax.set_aspect("equal")
    ax.legend(frameon=False)
    ax.set_title("Tool-centre-point path (side view, base frame)")
    fig.tight_layout()
    fig.savefig(site / "tcp.png", dpi=110)
    plt.close(fig)

    # ---- streaming step: tracking lag vs the ideal (mock) backend -------------------------------------------
    grid = np.arange(0, 6.0, 0.005)
    stream = {}
    for b, r in runs.items():
        s = next(x for x in r["steps"] if x["level"] == "streaming")
        # streaming starts after the controller switch: take the window and subtract its starting pose
        m = (r["t"] >= s["t_start"]) & (r["t"] <= s["t_end"])
        tt, qq = r["t"][m] - s["t_start"], r["q"][m]
        stream[b] = np.array([np.interp(grid, tt, qq[:, j] - qq[0, j]) for j in (0, 1, 5)])
    ref = stream["mock"]
    lag = {}
    for b, y in stream.items():
        best = (1e9, 0.0)
        for k in range(-100, 101):  # +/- 0.5 s
            a, c = (ref[0][max(0, -k):len(grid) - max(0, k)], y[0][max(0, k):len(grid) - max(0, -k)])
            err = float(np.sqrt(np.mean((a - c) ** 2)))
            best = min(best, (err, k * 0.005))
        lag[b] = {"rms_rad": best[0], "lag_ms": best[1] * 1000}
    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    for ax, idx, nm in zip(axes, range(3), ("joint1", "joint2", "joint6")):
        for b, y in stream.items():
            ax.plot(grid, y[idx], color=COLOR[b], lw=1.3, label=LABEL[b], alpha=0.9)
        ax.set_ylabel(f"{nm} change (rad)")
    axes[0].legend(frameon=False, ncol=4, loc="upper right")
    axes[-1].set_xlabel("time since streaming started (s)")
    fig.suptitle("Streamed 50 Hz joint targets: each backend's response", y=0.995)
    fig.tight_layout()
    fig.savefig(site / "stream.png", dpi=110)
    plt.close(fig)

    # ---- agreement at the end of each step, relative to mock -------------------------------------------------
    steps = runs["mock"]["steps"]
    agree = []
    for i, st in enumerate(steps):
        row = {"name": st["name"]}
        ref_q, ref_tcp = at(runs["mock"], st["t_end"]), None
        for b, r in runs.items():
            if b == "mock":
                continue
            sb = r["steps"][i]
            q = at(r, sb["t_end"])
            row[b] = (float(np.nanmax(np.abs(q[:6] - ref_q[:6]))),
                      float(np.linalg.norm(model.fk(dict(zip(ARM, q[:6])), "tcp_link")[:3, 3]
                                           - model.fk(dict(zip(ARM, ref_q[:6])), "tcp_link")[:3, 3]) * 1000),
                      float(abs(q[6] - ref_q[6]) * 1000) if not np.isnan(q[6]) else float("nan"))
        agree.append(row)

    # ---- media --------------------------------------------------------------------------------------------------
    import imageio_ffmpeg
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    for f in ("isaac.mp4", "mujoco.mp4", "gazebo_overview.mp4", "gazebo_wrist.mp4", "mujoco_wrist.mp4", "isaac_wrist.mp4"):
        if (demo / f).exists():  # faststart: moov atom first, so browsers can play and seek without Range support
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", str(demo / f),
                            "-c", "copy", "-movflags", "+faststart", str(site / f)], check=True)
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-ss", "4", "-i", str(demo / f), "-frames:v", "1",
                            str(site / (f[:-4] + "_poster.png"))], check=True)
    for f in ("gazebo_wrist.png", "mujoco_wrist.png", "isaac_wrist.png"):
        if (demo / f).exists():
            shutil.copy(demo / f, site / f)

    # ---- html ------------------------------------------------------------------------------------------------------
    def cell(b, i):
        s = runs[b]["steps"][i]
        ok = "ok" if s["success"] else "bad"
        return f'<td class="{ok}">{"✓" if s["success"] else "✗"} <span class="muted">{s["t_end"] - s["t_start"]:.1f} s</span></td>'

    rows = "\n".join(
        f'<tr><td>{html.escape(st["name"])}<div class="muted">{html.escape(st["level"])}</div></td>'
        + "".join(cell(b, i) for b in runs) + "</tr>" for i, st in enumerate(steps))
    head = "".join(f"<th>{LABEL[b]}</th>" for b in runs)

    def fmt(v):
        return "—" if v != v else f"{v:.3f}" if v < 1 else f"{v:.1f}"

    arows = "\n".join(
        f'<tr><td>{html.escape(a["name"])}</td>'
        + "".join(f'<td>{fmt(a[b][0])}</td><td>{fmt(a[b][1])}</td>' for b in runs if b != "mock") + "</tr>" for a in agree)
    ahead = "".join(f'<th colspan="2">{LABEL[b]}</th>' for b in runs if b != "mock")
    asub = "".join("<th>joints (rad)</th><th>TCP (mm)</th>" for b in runs if b != "mock")
    worst = {b: (max(a[b][0] for a in agree), max(a[b][1] for a in agree)) for b in runs if b != "mock"}
    lagrows = "".join(
        f'<tr><td>{LABEL[b]}</td><td>{lag[b]["lag_ms"]:+.0f} ms</td><td>{lag[b]["rms_rad"] * 1000:.1f} mrad</td></tr>'
        for b in runs)
    try:
        commit = subprocess.check_output(["git", "-C", str(Path(__file__).parent), "rev-parse", "--short", "HEAD"], text=True).strip()
        branch = subprocess.check_output(["git", "-C", str(Path(__file__).parent), "branch", "--show-current"], text=True).strip()
    except Exception:
        commit, branch = "?", "?"
    video = lambda f, title, cap: (  # noqa: E731
        f'<figure><video controls preload="metadata" poster="{f[:-4]}_poster.png" src="{f}"></video>'
        f"<figcaption><b>{title}</b><br>{cap}</figcaption></figure>") if (site / f).exists() else ""
    n_ok = {b: sum(s["success"] for s in r["steps"]) for b, r in runs.items()}
    page = TEMPLATE.format(
        date=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), commit=commit, branch=branch,
        summary=" · ".join(f"{LABEL[b]} {n_ok[b]}/{len(runs[b]['steps'])}" for b in runs),
        videos=video("gazebo_overview.mp4", "Gazebo Harmonic, observer camera inside the simulator",
                     "A fixed camera sensor rendered by Gazebo itself (EGL, headless) while the demo runs.")
        + video("isaac.mp4", "Isaac Sim 6.1, rendered live while the ROS side drives it",
                "MoveIt plans and ros2_control tracks, exactly like every other backend; Isaac runs as its own headless process.")
        + video("mujoco.mp4", "MuJoCo, replay of the recorded run",
                "The generated MJCF replayed from the recorded joint states (grey: collision meshes are used as visuals)."),
        nav=site_pages.nav("index.html"), navcss=site_pages.CSS.split("nav {")[1].split("h1 {")[0],
        head=head, rows=rows, ahead=ahead, asub=asub, arows=arows, lagrows=lagrows,
        worst="; ".join(f"{LABEL[b]}: {w[0]:.3f} rad / {w[1]:.1f} mm" for b, w in worst.items()))
    (site / "index.html").write_text(page)
    print("wrote", site / "index.html")

    meta = f"{datetime.datetime.now().strftime('%Y-%m-%d %H:%M')} · branch <code>{branch}</code> @ <code>{commit}</code> · simulation only, real arm not involved"
    facts = json.loads((demo / "facts.json").read_text()) if (demo / "facts.json").exists() else {}
    (site / "status.html").write_text(site_pages.status_page(facts, meta))
    (site / "control.html").write_text(site_pages.control_page(meta))
    (site / "architecture.html").write_text(site_pages.architecture_page(meta))
    (site / "camera.html").write_text(site_pages.camera_page(site, meta))
    print("wrote status, control, architecture, camera pages")


TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Piper Studio Demo</title>
<style>
nav {{{navcss}
:root {{ --bg:#f7f7f5; --card:#fff; --ink:#1c1e21; --muted:#6b7280; --line:#e5e7eb; --ok:#15803d; --bad:#b91c1c; --accent:#2563eb; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#0f1115; --card:#181b21; --ink:#e6e8eb; --muted:#9aa3af; --line:#2a2f38; --ok:#4ade80; --bad:#f87171; --accent:#60a5fa; }} }}
* {{ box-sizing:border-box; }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:15px/1.55 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }}
main {{ max-width:1080px; margin:0 auto; padding:24px 16px 64px; }}
h1 {{ font-size:28px; margin:0 0 4px; }} h2 {{ font-size:20px; margin:36px 0 10px; }}
.muted {{ color:var(--muted); font-size:13px; }} .lead {{ font-size:17px; max-width:70ch; }}
.card {{ background:var(--card); border:1px solid var(--line); border-radius:12px; padding:16px; overflow-x:auto; }}
.videos {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(340px,1fr)); gap:16px; }}
figure {{ margin:0; background:var(--card); border:1px solid var(--line); border-radius:12px; overflow:hidden; }}
video {{ width:100%; display:block; background:#000; }} figcaption {{ padding:10px 14px; font-size:14px; }}
table {{ border-collapse:collapse; width:100%; font-size:14px; }} th,td {{ padding:7px 10px; border-bottom:1px solid var(--line); text-align:left; }}
th {{ font-weight:600; }} td.ok {{ color:var(--ok); font-weight:600; }} td.bad {{ color:var(--bad); font-weight:600; }}
img {{ max-width:100%; background:#fff; border-radius:8px; }} .grid2 {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(360px,1fr)); gap:16px; align-items:start; }}
code, pre {{ font:13px ui-monospace,SFMono-Regular,Menlo,monospace; }} pre {{ background:var(--bg); border:1px solid var(--line); border-radius:8px; padding:12px; overflow-x:auto; }}
ul {{ padding-left:20px; }} li {{ margin:4px 0; }} .pill {{ display:inline-block; padding:2px 10px; border-radius:999px; background:var(--card); border:1px solid var(--line); font-size:13px; }}
</style></head><body><main>
{nav}
<h1>Piper Studio: simulation demo</h1>
<div class="muted">{date} · branch <code>{branch}</code> @ <code>{commit}</code> · real arm not involved</div>
<p class="lead">One scripted sequence, run unchanged on four backends through the same controllers, MoveIt configuration and Python API:
planning, straight-line moves, gripper, a direct trajectory and 50&nbsp;Hz streaming. <span class="pill">{summary}</span></p>

<h2>Videos</h2>
<div class="videos">{videos}</div>

<h2>The sequence on every backend</h2>
<div class="card"><table><tr><th>Step</th>{head}</tr>{rows}</table></div>
<p class="muted">✓ = the controller reported the goal reached (not just planned). Times are simulated seconds per step.</p>

<h2>Do the backends agree?</h2>
<p>State at the end of each step compared with the mock backend (ideal tracking). Both the largest joint difference and the tool-centre-point (fingertip) difference are shown.
Worst case over the sequence: {worst}.</p>
<div class="card"><table><tr><th rowspan="2">Step</th>{ahead}</tr><tr>{asub}</tr>{arows}</table></div>
<div class="grid2" style="margin-top:16px">
<div class="card"><img src="tcp.png" alt="TCP path"></div>
<div class="card"><h3 style="margin-top:0">Streaming response vs. ideal</h3>
<table><tr><th>Backend</th><th>Lag of joint1</th><th>Residual RMS</th></tr>{lagrows}</table>
<p class="muted">Lag found by sliding each trace against the mock trace (±0.5 s); residual is what remains. The
servo models of MuJoCo and Isaac are provisional and equal; Gazebo's position control is kinematic.</p></div>
</div>
<div class="card" style="margin-top:16px"><img src="stream.png" alt="Streaming response"></div>
<div class="card" style="margin-top:16px"><img src="joints.png" alt="Joint trajectories"></div>

<h2>What this does and does not show</h2>
<ul>
<li><b>Shows:</b> the same ROS interfaces (controllers, MoveIt, <code>piper_py</code>) work on four backends, the simulator models are generated from one official description, and failures are reported as failures.</li>
<li><b>Does not show:</b> real-arm behaviour. Simulator actuator models are provisional (not identified on the arm), joint efforts in the URDF are placeholders, and the gripper stroke on the real arm is unmeasured. Nothing here has moved the real Piper.</li>
<li>The 14 automated checks (description vs. official model, generated MuJoCo kinematics, end-to-end on each backend, real-arm plumbing with a stand-in driver) pass; Isaac's end-to-end test is opt-in because it uses the shared GPU.</li>
</ul>

<h2>Reproduce</h2>
<pre>git clone --recursive git@github.com:charithmu/piper_studio.git &amp;&amp; cd piper_studio
scripts/bootstrap.sh &amp;&amp; source scripts/env.sh
scripts/isaac.sh build-usd                     # once, for Isaac (see isaac/README.md)
scripts/run_demo.sh mujoco  out/               # also: mock | gazebo | isaac
python tools/build_demo_page.py out/ site/</pre>
</main></body></html>
"""

if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
