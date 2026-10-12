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
    p.add_argument("--camera", default="none", help="none | d435 (wrist RealSense)")
    p.add_argument("--camera-pose-out", type=Path,
                   help="with --camera: write the colour camera pose in link6 as JSON, for a simulator that creates the camera itself "
                        "(USD convention: the camera looks along -Z with +Y up, i.e. the optical frame turned 180 deg about x)")
    a = p.parse_args()
    cam = {"camera": a.camera} if a.camera != "none" else {}
    urdf = robot_description(a.hardware, gripper=not a.no_gripper, **cam)
    if a.physics:
        urdf = physics_model(urdf, keep_mimic=a.keep_mimic)

    def resolve(m):
        return f'filename="{get_package_share_directory(m.group(1))}/{m.group(2)}"'

    urdf = re.sub(r'filename="package://([^/]+)/([^"]+)"', resolve, urdf)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(urdf)
    print(a.out)
    if a.camera != "none" and a.camera_pose_out:
        import json

        import model_audit
        import numpy as np
        from scipy.spatial.transform import Rotation

        m = model_audit.load_urdf("camera", a.out)
        rel = np.linalg.inv(m.fk({}, "link6")) @ m.fk({}, "camera_color_optical_frame")
        rot = Rotation.from_matrix(rel[:3, :3] @ np.diag([1.0, -1.0, -1.0]))
        a.camera_pose_out.write_text(json.dumps({"link": "link6", "pos": rel[:3, 3].tolist(), "quat_xyzw": rot.as_quat().tolist(),
                                                 "width": 640, "height": 480, "hfov_deg": 69.4, "near": 0.05, "far": 6.0}))
        print(a.camera_pose_out)


if __name__ == "__main__":
    main()
