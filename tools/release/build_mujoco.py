#!/usr/bin/env python3
"""Build the data of the `piper_mujoco` wheel: the MuJoCo model of the arm (no prefix) with the ZED X Mini wrist camera (the default) and without it, each with its two scenes.

    source scripts/env.sh && python tools/release/build_mujoco.py [--out src/piper_mujoco/piper_mujoco/data]

The model is the one every simulator run uses: piper_sim.mujoco_model() converts the same description with mujoco_ros2_control's converter and
the same inputs.xml (position servos, gravity compensation, mimic fingers), so PiperSim and the ROS MuJoCo backend share one actuator model.
  model/ and model_no_camera/   converter output (mujoco_description_formatted.xml, assets/*.obj); the scenes sit next to it (meshdir is relative)
  model/scene_empty.xml  floor plane + arm
  model/scene_table.xml        placeholder table (unmeasured) + the shared scene objects as free bodies + arm
  manifest.json          source commit and file hashes
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

import piper_sim
from piper_description import camera_model

WS = Path(__file__).resolve().parents[2]
INCLUDE = '<include file="mujoco_description_formatted.xml"/>'


def body(o: dict) -> str:
    x, y, z = o["pos"]
    rgba = " ".join(f"{v:g}" for v in o["rgba"])
    if o["shape"] == "box":
        geom = f'type="box" size="{" ".join(f"{s / 2:g}" for s in o["size"])}"'
    else:
        r, h = o["size"]
        geom = f'type="cylinder" size="{r:g} {h / 2:g}"'
    return (f'    <body name="{o["name"]}" pos="{x:g} {y:g} {z:g}"><freejoint name="{o["name"]}"/>'
            f'<geom name="{o["name"]}" {geom} mass="0.05" rgba="{rgba}" friction="1 0.005 0.0001"/></body>')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=WS / "src/piper_mujoco/piper_mujoco/data")
    out = ap.parse_args().out.resolve()
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    cfg = Path(get_package_share_directory("piper_sim")) / "config" / "mujoco"
    base = (cfg / "scene.xml").read_text()
    for name, camera in (("model", "zedxm"), ("model_no_camera", "none")):
        build_variant(out / name, camera, cfg, base)
    spec = yaml.safe_load((WS / "src/piper_sim/config/scene_objects.yaml").read_text())
    for name in ("model", "model_no_camera"):
        add_scenes(out / name, base, spec)
    finish(out)


def build_variant(model: Path, camera: str, cfg: Path, base: str):
    with tempfile.TemporaryDirectory() as tmp:
        scene = piper_sim.mujoco_model(piper_sim.robot_description("mujoco", prefix="", camera=camera), cfg / "inputs.xml", cfg / "scene.xml", cache_dir=tmp, prefix="",
                                       camera=camera_model() if camera != "none" else None)
        model.mkdir()
        shutil.copytree(scene.parent / "assets", model / "assets")
        shutil.copy2(scene.parent / "mujoco_description_formatted.xml", model / "mujoco_description_formatted.xml")


def add_scenes(model: Path, base: str, spec: dict):
    assert INCLUDE in base and "</worldbody>" in base
    (model / "scene_empty.xml").write_text(base)

    t = spec["table"]
    sx, sy, sz = t["size"]
    cx, cy = t["pos_xy"]
    table = (f'    <!-- PLACEHOLDER table, unmeasured (config/scene_objects.yaml in piper_sim) -->\n'
             f'    <geom name="table" type="box" size="{sx / 2:g} {sy / 2:g} {sz / 2:g}" pos="{cx:g} {cy:g} {t["top_z"] - sz / 2:g}" rgba="0.55 0.45 0.35 1" friction="1 0.005 0.0001"/>')
    text = base
    text = text.replace('<geom name="floor" size="0 0 0.05"', '<geom name="floor" pos="0 0 -0.5" size="0 0 0.05"')     # far below: objects that fall off the table land there
    text = text.replace("</worldbody>", table + "\n" + "\n".join(body(o) for o in spec["objects"]) + "\n  </worldbody>", 1)
    (model / "scene_table.xml").write_text(text)


def finish(out: Path):
    def run(*a):
        return subprocess.run(["git", "-C", str(WS), *a], capture_output=True, text=True).stdout.strip()
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob("*")) if p.is_file()}
    (out / "manifest.json").write_text(json.dumps({"source": {"commit": run("rev-parse", "HEAD"), "describe": run("describe", "--tags", "--always", "--dirty")},
                                                    "scenes": ["empty", "table"], "files": files}, indent=1))
    print(f"{out}: {len(files)} files, {sum(p.stat().st_size for p in out.rglob('*') if p.is_file()) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
