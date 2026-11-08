#!/usr/bin/env python3
"""Build meshes/d435_colored.dae: the D435 with real part colours, in the same frame as realsense2_description's d435.dae.

Source: MuJoCo Menagerie `realsense_d435i` (Apache-2.0, Google DeepMind), parts and rgba from its d435i.xml. The part
geometry matches realsense2_description's mesh (same bounding boxes), so it is a drop-in replacement for the grey one.

    python tools/build_d435_mesh.py [MENAGERIE_DIR] [OUT.dae]
"""
import re
import sys
from pathlib import Path

import numpy as np
import trimesh

src = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / ".cache/robot_descriptions/mujoco_menagerie/realsense_d435i")
out = Path(sys.argv[2] if len(sys.argv) > 2 else Path(__file__).resolve().parents[1] / "meshes/d435_colored.dae")

# part index -> material rgba (from d435i.xml)
RGBA = {
    0: (0.070360, 0.070360, 0.070360), 1: (0.287440, 0.665387, 0.327778), 2: (0.799102, 0.806952, 0.799103),
    3: (0.035601, 0.035601, 0.035601), 4: (0.296138, 0.296138, 0.296138), 5: (0.070360, 0.070360, 0.070360),
    6: (0.070360, 0.070360, 0.070360), 7: (0.087140, 0.002866, 0.009346), 8: (1.0, 1.0, 1.0),
}
RGBA[0] = (0.035601,) * 3  # d435i.xml: parts 0 and 3 are IR_Lens
parts = []
for i, rgb in RGBA.items():
    m = trimesh.load(src / "assets" / f"d435i_{i}.obj", force="mesh", process=True)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(np.array([*rgb, 1.0]) * 255, (len(m.faces), 1)).astype(np.uint8))
    m.metadata["name"] = f"d435_part{i}"
    parts.append(m)
text = trimesh.exchange.dae.export_collada(parts).decode()
text = re.sub(r"-?\d+\.\d{7,}(?:e-?\d+)?", lambda m: f"{float(m.group()):.6g}", text)  # trim float noise: ~3x smaller
# drop the per-face-corner normals (they are 2/3 of the file); loaders regenerate smooth normals
text = re.sub(r'<source id="normals-array">.*?</source>\s*', "", text, flags=re.S)
text = re.sub(r'\s*<input offset="1" semantic="NORMAL"[^>]*/>', "", text)
text = re.sub(r"(<p>)([^<]*)(</p>)", lambda m: m.group(1) + " ".join(m.group(2).split()[::2]) + m.group(3), text)
out.write_text(text)
print(out, out.stat().st_size // 1024, "KiB")
