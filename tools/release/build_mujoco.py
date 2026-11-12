#!/usr/bin/env python3
"""Build the data of the `piper_mujoco` wheel: the MuJoCo model of the arm (no prefix) with the ZED X Mini wrist camera (the default) and without it, each with its two scenes.

    source scripts/env.sh && python tools/release/build_mujoco.py [--out src/piper_mujoco/piper_mujoco/data]

The model is the one every simulator run uses: piper_sim.mujoco_model() converts the same description with mujoco_ros2_control's converter and
the same inputs.xml (position servos, gravity compensation, mimic fingers), so PiperSim and the ROS MuJoCo backend share one actuator model.
  model/ and model_no_camera/   converter output (mujoco_description_formatted.xml, assets/*.obj); the scenes sit next to it (meshdir is relative)
  model_go2/             the full setup on the Go2 (nominal mount): the dog fixed, posed standing (leg joints frozen at config/go2 stand pose), held at the height that puts its feet on the floor; its collision is the ONE
                         generated box of the planner's world (tools/go2_cover) + the plate (the converter's mesh collisions of the dog are removed), its visual the Go2's meshes (STL, one grey); the arm has the same joints, servos and camera as the other models. Scene: empty only.
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
GO2_PKG = Path.home() / "projects/ros/go2_studio/third_party/unitree_ros/robots/go2_description"      # external input: the Go2 description (Unitree, BSD-3)
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
    setups = ["arm", "arm_cam"]
    if GO2_PKG.exists():
        build_go2(out / "model_go2", cfg, base)
        setups.append("go2")
    finish(out, setups)


def build_variant(model: Path, camera: str, cfg: Path, base: str):
    with tempfile.TemporaryDirectory() as tmp:
        scene = piper_sim.mujoco_model(piper_sim.robot_description("mujoco", prefix="", camera=camera), cfg / "inputs.xml", cfg / "scene.xml", cache_dir=tmp, prefix="",
                                       camera=camera_model() if camera != "none" else None)
        model.mkdir()
        shutil.copytree(scene.parent / "assets", model / "assets")
        shutil.copy2(scene.parent / "mujoco_description_formatted.xml", model / "mujoco_description_formatted.xml")


def go2_urdf_for_mujoco(stl_dir: Path) -> str:
    """The Go2 + Piper URDF for the converter: legs frozen at the stand pose, the dog held at its floor height, the dog's visual meshes as STL
    (the converter cannot take the Go2's .dae: several materials per visual), one grey material, virtual inertials for the arm's massless moving links."""
    import logging
    import xml.etree.ElementTree as ET

    import trimesh
    from piper_description import cover_report, freeze_legs, go2_piper_description
    logging.getLogger("trimesh").setLevel(logging.ERROR)
    root = ET.fromstring(freeze_legs(go2_piper_description(prefix="", camera="zedxm", hardware="mujoco")))
    used, first = set(cover_report()["links_used"]), True
    stl_dir.mkdir(parents=True, exist_ok=True)
    for link in root.findall("link"):
        if link.get("name") not in used:
            continue
        for v in link.findall("visual"):
            for m in v.findall("material"):
                v.remove(m)
            mesh = v.find("geometry/mesh")
            if mesh is None:
                continue
            stem = Path(mesh.get("filename")).stem
            out = stl_dir / f"go2_{stem}.stl"
            if not out.exists():
                trimesh.load(GO2_PKG / "dae" / f"{stem}.dae", force="mesh").export(out)
            mesh.set("filename", str(out))
            mat = ET.SubElement(v, "material", name="go2_grey")
            if first:
                ET.SubElement(mat, "color", rgba="0.67 0.69 0.77 1")
                first = False
    return piper_sim.add_virtual_inertials(ET.tostring(root, encoding="unicode"))


def dog_collision_is_the_cover(xml_file: Path):
    """The converter drops every URDF <collision> and turns each visual into a collision geom too, so the dog's meshes would collide. Remove those collision copies of the Go2's meshes and add the ONE
    generated box of the planner's world (tools/go2_cover) as the dog's collision, in the `world` body the dog is fused into; the dog's visual meshes stay. The plate keeps its own box."""
    import xml.etree.ElementTree as ET

    from piper_description import cover_report, dog_cover
    tree = ET.parse(xml_file)
    world = tree.getroot().find("worldbody")
    go2_meshes = {el.get("mesh") for el in world.findall("geom") if (el.get("mesh") or "").startswith("go2_")}
    removed = 0
    for geom in list(world.findall("geom")):
        if geom.get("class") == "collision" and geom.get("mesh") in go2_meshes:
            world.remove(geom)
            removed += 1
    h = -cover_report()["lowest_point_z_m"]                      # the dog is held this high over the floor (freeze_legs)
    dog = dog_cover("base")["dog"]
    x, y, z = dog["pose"][:3]
    ET.SubElement(world, "geom", {"name": "dog_cover", "type": "box", "class": "collision", "size": " ".join(f"{v / 2:.6f}" for v in dog["dims"]), "pos": f"{x:.6f} {y:.6f} {z + h:.6f}"})
    ET.indent(tree)
    tree.write(xml_file)
    return removed


def build_go2(model: Path, cfg: Path, base: str):
    with tempfile.TemporaryDirectory() as tmp:
        scene = piper_sim.mujoco_model(go2_urdf_for_mujoco(Path(tmp) / "stl"), cfg / "inputs.xml", cfg / "scene.xml", cache_dir=Path(tmp) / "mj", prefix="", camera=camera_model())
        model.mkdir()
        shutil.copytree(scene.parent / "assets", model / "assets")
        shutil.copy2(scene.parent / "mujoco_description_formatted.xml", model / "mujoco_description_formatted.xml")
    assert dog_collision_is_the_cover(model / "mujoco_description_formatted.xml") >= 10, "the converter no longer makes collision copies of the dog's meshes: revisit"
    (model / "scene_empty.xml").write_text(base)


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


def finish(out: Path, setups=("arm", "arm_cam")):
    def run(*a):
        return subprocess.run(["git", "-C", str(WS), *a], capture_output=True, text=True).stdout.strip()
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob("*")) if p.is_file()}
    (out / "manifest.json").write_text(json.dumps({"source": {"commit": run("rev-parse", "HEAD"), "describe": run("describe", "--tags", "--always", "--dirty")},
                                                    "scenes": ["empty", "table"], "setups": list(setups), "go2_scenes": ["empty"] if "go2" in setups else [], "files": files}, indent=1))
    print(f"{out}: {len(files)} files, {sum(p.stat().st_size for p in out.rglob('*') if p.is_file()) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
