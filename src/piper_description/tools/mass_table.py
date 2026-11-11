#!/usr/bin/env python3
"""Print the mass of every link of the Piper description (arm, gripper, camera) as a markdown table, with where each value comes from.

    python src/piper_description/tools/mass_table.py            # to the terminal
    python src/piper_description/tools/mass_table.py > docs/description/MASSES.md   # (add the header text by hand)

Sources (the table says which applies): the vendor URDF (agx_arm_urdf, CAD estimates, not measured on the real arm), or a placeholder
of ours. The firmware does not report masses.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from piper_description import robot_description  # noqa: E402

VENDOR = "vendor URDF (agx_arm_urdf, CAD estimate)"
OURS = {"camera_mount_link": "PLA estimate from the mesh volume (mounts/zedxm_bracket.xacro), not weighed",
        "camera_link": "cameras/zedxm.xacro: 150 g from the Stereolabs datasheet"}
NOTE = {"gripper_link": "virtual width joint, no mass"}


def main() -> int:
    root = ET.fromstring(robot_description("mock", prefix=""))
    rows, groups = [], {"arm": 0.0, "gripper": 0.0, "camera": 0.0}
    for link in root.findall("link"):
        n = link.get("name")
        m = link.find("inertial/mass")
        mass = float(m.get("value")) if m is not None else 0.0
        if mass == 0.0 and n not in NOTE:
            continue
        group = "camera" if n.startswith("camera") else "gripper" if n.startswith("gripper") or n == "flange_link" else "arm"
        groups[group] += mass
        rows.append((n, mass, OURS.get(n, VENDOR) if group != "arm" or n in OURS else VENDOR, NOTE.get(n, "")))
    print("| Link | Mass (kg) | Source | Note |\n|---|---:|---|---|")
    for n, mass, src, note in rows:
        print(f"| `{n}` | {mass:.5g} | {src} | {note} |")
    print(f"| **arm (base to link6)** | **{groups['arm']:.4f}** | sum | |")
    print(f"| **flange + gripper** | **{groups['gripper']:.4f}** | sum | |")
    print(f"| **camera + stand** | **{groups['camera']:.4f}** | sum | |")
    print(f"| **total with gripper and camera** | **{sum(groups.values()):.4f}** | sum | |")
    print(f"| **total arm + gripper (no camera)** | **{groups['arm'] + groups['gripper']:.4f}** | sum | |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
