#!/usr/bin/env python3
"""Move a running backend to the 'ready' pose and save its wrist-camera view (colour over depth) as a PNG.

    python tools/capture_stills.py OUT.png        # needs a launch with camera:=d435; prints the camera rates too
"""
import sys
import time

import imageio.v2 as imageio
import numpy as np
import rclpy
from sensor_msgs.msg import Image

from piper_py import Piper

READY = [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]

with Piper() as arm:
    arm.wait_ready(120)
    for _ in range(150):
        if arm.controllers().get("arm_controller") == "active":
            break
        time.sleep(0.2)
    print("move to ready:", arm.move_joints(READY, 3.0).code)
    time.sleep(1.5)
    color, _ = arm.image("color", 60)
    depth, _ = arm.image("depth", 60)
    valid = np.isfinite(depth) & (depth > 0)
    dv = np.where(valid, 255 * (1 - np.clip(depth / 0.8, 0, 1)), 0).astype(np.uint8)
    imageio.imwrite(sys.argv[1], np.concatenate([color, np.repeat(dv[:, :, None], 3, 2)], 0))
    print("depth valid %.0f%%, median %.2f m" % (100 * valid.mean(), float(np.median(depth[valid])) if valid.any() else float("nan")))
    n = {"c": 0}
    arm.node.create_subscription(Image, "/camera/color/image_raw", lambda m: n.__setitem__("c", n["c"] + 1), 10)
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rclpy.spin_once(arm.node, timeout_sec=0.1)
    print("colour image rate: %.1f Hz (wall clock)" % (n["c"] / 4.0))
