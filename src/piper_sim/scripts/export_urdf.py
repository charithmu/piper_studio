#!/usr/bin/env python3
"""Write the Piper description for a backend as a standalone URDF (no ROS needed to read it).

    ros2 run piper_sim export_urdf.py --hardware isaac --physics --out piper_isaac.urdf

--physics   apply physics_model(): widened joint limits, no mimic unless --keep-mimic
--gripper-width W   hold the gripper at W metres (0 to 0.07): the gripper and finger joints become fixed (for planners such as cuRobo)
Mesh references are rewritten from package:// to absolute file paths so non-ROS tools (Isaac Sim's
URDF importer) can resolve them.
"""

import argparse
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from piper_description import fixed_gripper
from piper_sim import physics_model, robot_description


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--hardware", default="isaac")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--physics", action="store_true")
    p.add_argument("--keep-mimic", action="store_true", help="with --physics: keep URDF mimic joints")
    p.add_argument("--no-gripper", action="store_true")
    p.add_argument("--gripper-width", type=float, default=None, help="fix the gripper at this opening in metres (0 closed, 0.07 open)")
    p.add_argument("--camera", default="zedxm", help="zedxm (the ZED X Mini wrist camera on the own bracket, default) | none (the robot only)")
    p.add_argument("--camera-calibration", default="", help="a version of the bracket (a name in config/camera_mounts, e.g. zedxm_bracket_v2) or a yaml path (default: the newest)")
    p.add_argument("--camera-lens", default="", help="wide | narrow (default: config/camera_models/zedxm.yaml)")
    p.add_argument("--camera-width", type=int, default=0, help="simulated image width, px (default: the camera model's)")
    p.add_argument("--camera-height", type=int, default=0, help="simulated image height, px (default: the camera model's)")
    p.add_argument("--prefix", default="piper_", help="prefix of every link, joint and frame name")
    p.add_argument("--camera-pose-out", type=Path,
                   help="with --camera: write the colour camera pose in link6 as JSON, for a simulator that creates the camera itself "
                        "(USD convention: the camera looks along -Z with +Y up, i.e. the optical frame turned 180 deg about x)")
    a = p.parse_args()
    cam = {"camera": a.camera}
    if a.camera != "none":
        cam.update({k: v for k, v in (("camera_calibration", a.camera_calibration), ("camera_lens", a.camera_lens),
                                      ("camera_width", a.camera_width or ""), ("camera_height", a.camera_height or "")) if v})
    urdf = robot_description(a.hardware, gripper=not a.no_gripper, prefix=a.prefix, **cam)
    if a.gripper_width is not None:
        urdf = fixed_gripper(urdf, a.gripper_width, a.prefix)
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

        import sys

        from ament_index_python.packages import get_package_prefix
        sys.path.insert(0, str(Path(get_package_prefix("piper_description")) / "lib" / "piper_description"))
        import model_audit  # installed by piper_description
        import numpy as np
        from scipy.spatial.transform import Rotation

        from piper_description import camera_model
        cm = camera_model(a.camera_lens or None, a.camera_width or None, a.camera_height or None)
        m = model_audit.load_urdf("camera", a.out)
        rel = np.linalg.inv(m.fk({}, a.prefix + "link6")) @ m.fk({}, a.prefix + "zed_left_camera_frame_optical")
        rot = Rotation.from_matrix(rel[:3, :3] @ np.diag([1.0, -1.0, -1.0]))
        a.camera_pose_out.write_text(json.dumps({"link": a.prefix + "link6", "pos": rel[:3, 3].tolist(), "quat_xyzw": rot.as_quat().tolist(),
                                                 "width": cm["width"], "height": cm["height"], "hfov_deg": cm["hfov_deg"],
                                                 "near": cm["depth_min"], "far": cm["depth_max"], "K": cm["K"]}))
        print(a.camera_pose_out)


if __name__ == "__main__":
    main()
