"""Static scene objects shared by the simulators (config/scene_objects.yaml in piper_bringup).

One list of boxes/cylinders in the base frame, rendered into each simulator's native format, so a wrist-camera frame
from Gazebo, MuJoCo and Isaac shows the same world.
"""

from __future__ import annotations

from pathlib import Path

import yaml


def load_objects(path: str | Path) -> list[dict]:
    return yaml.safe_load(Path(path).read_text())["objects"]


def mujoco_geoms(objects: list[dict]) -> str:
    """<geom> elements for the worldbody of the MJCF scene (static, collidable)."""
    out = []
    for o in objects:
        x, y, z = o["pos"]
        rgba = " ".join(f"{v:g}" for v in o["rgba"])
        if o["shape"] == "box":
            half = " ".join(f"{s / 2:g}" for s in o["size"])
            out.append(f'    <geom name="{o["name"]}" type="box" size="{half}" pos="{x:g} {y:g} {z:g}" rgba="{rgba}"/>')
        elif o["shape"] == "cylinder":
            r, h = o["size"]
            out.append(f'    <geom name="{o["name"]}" type="cylinder" size="{r:g} {h / 2:g}" pos="{x:g} {y:g} {z:g}" rgba="{rgba}"/>')
        else:
            raise ValueError(f"unknown shape {o['shape']!r}")
    return "\n".join(out)


def mujoco_scene(base_scene: str | Path, objects: list[dict]) -> str:
    """Base scene XML text with the objects added to its worldbody."""
    text = Path(base_scene).read_text()
    if "</worldbody>" not in text:
        raise ValueError("scene.xml has no <worldbody>")
    return text.replace("</worldbody>", mujoco_geoms(objects) + "\n  </worldbody>", 1)


def sdf_models(objects: list[dict]) -> str:
    """<model> elements for an SDF world (static)."""
    out = []
    for o in objects:
        x, y, z = o["pos"]
        r, g, b, a = o["rgba"]
        if o["shape"] == "box":
            geom = f'<box><size>{" ".join(f"{s:g}" for s in o["size"])}</size></box>'
        elif o["shape"] == "cylinder":
            geom = f'<cylinder><radius>{o["size"][0]:g}</radius><length>{o["size"][1]:g}</length></cylinder>'
        else:
            raise ValueError(f"unknown shape {o['shape']!r}")
        mat = f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>"
        out.append(f'''    <model name="{o["name"]}"><static>true</static><pose>{x:g} {y:g} {z:g} 0 0 0</pose>
      <link name="link"><collision name="c"><geometry>{geom}</geometry></collision>
        <visual name="v"><geometry>{geom}</geometry>{mat}</visual></link></model>''')
    return "\n".join(out)


GAZEBO_WORLD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="piper_world">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.6 0.75 0.9 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 5 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.3 -0.8</direction></light>
    <model name="ground"><static>true</static><pose>0 0 -0.05 0 0 0</pose><link name="link">
      <collision name="c"><geometry><box><size>20 20 0.1</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>20 20 0.1</size></box></geometry>
        <material><ambient>0.45 0.5 0.55 1</ambient><diffuse>0.45 0.5 0.55 1</diffuse></material></visual></link></model>
{objects}
{overview}  </world>
</sdf>
"""


# Fixed observer camera (same viewpoint as tools/render_mujoco_video.py); publishes gz topic overview/image.
OVERVIEW_CAMERA = """    <model name="overview_camera"><static>true</static><pose>0.945 -0.984 0.617 0 0.314 2.269</pose><link name="link">
      <sensor name="overview" type="camera"><always_on>1</always_on><update_rate>30</update_rate><topic>overview/image</topic>
        <camera><horizontal_fov>1.1</horizontal_fov><image><width>1280</width><height>720</height></image>
          <clip><near>0.1</near><far>20</far></clip></camera></sensor></link></model>
"""


def gazebo_world(objects: list[dict], overview: bool = False) -> str:
    return GAZEBO_WORLD.format(objects=sdf_models(objects), overview=OVERVIEW_CAMERA if overview else "")
