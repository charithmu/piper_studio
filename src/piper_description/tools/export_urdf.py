#!/usr/bin/env python3
"""Write the Piper description for a backend as a standalone URDF (no ROS needed to read it).

    ros2 run piper_description export_urdf.py --hardware isaac --physics --out piper_isaac.urdf

--physics   apply physics_model(): widened joint limits, no mimic unless --keep-mimic
Mesh references are rewritten from package:// to absolute file paths so non-ROS tools (Isaac Sim's
URDF importer) can resolve them.
"""

import argparse
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from piper_description import physics_model, robot_description


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hardware", default="isaac")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--physics", action="store_true")
    p.add_argument("--keep-mimic", action="store_true", help="with --physics: keep URDF mimic joints")
    p.add_argument("--no-gripper", action="store_true")
    a = p.parse_args()
    urdf = robot_description(a.hardware, gripper=not a.no_gripper)
    if a.physics:
        urdf = physics_model(urdf, keep_mimic=a.keep_mimic)

    def resolve(m):
        return f'filename="{get_package_share_directory(m.group(1))}/{m.group(2)}"'

    urdf = re.sub(r'filename="package://([^/]+)/([^"]+)"', resolve, urdf)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(urdf)
    print(a.out)


if __name__ == "__main__":
    main()
