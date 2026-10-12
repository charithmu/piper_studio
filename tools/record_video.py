#!/usr/bin/env python3
"""Record ROS image topics to mp4 files, paced by message stamps (so the video runs in simulation time).

    python tools/record_video.py /camera/color/image_raw=wrist.mp4 /overview/image_raw=overview.mp4 [--fps 20]

Runs until SIGINT/SIGTERM. Needs the workspace env.
"""
import argparse
import signal

import imageio.v2 as imageio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from piper_py.robot import _image_to_array


class Recorder(Node):
    def __init__(self, targets: dict[str, str], fps: int):
        super().__init__("record_video")
        self.fps, self.t_out, self.prev, self.writers = fps, {}, {}, {}
        for topic, path in targets.items():
            self.writers[topic] = imageio.get_writer(path, fps=fps, codec="libx264", quality=7, macro_block_size=2)
            self.t_out[topic], self.prev[topic] = None, None
            self.create_subscription(Image, topic, lambda m, t=topic: self.on_image(t, m), qos_profile_sensor_data)

    def on_image(self, topic: str, msg: Image):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame = _image_to_array(msg)
        if frame.ndim == 2:
            return  # depth is not recorded
        dt, t_out, w = 1.0 / self.fps, self.t_out[topic], self.writers[topic]
        if t_out is None:
            t_out = stamp
        if stamp < t_out - 0.5 * dt:
            return  # faster than the video rate: drop
        while t_out < stamp - 0.5 * dt and self.prev[topic] is not None:
            w.append_data(self.prev[topic])  # slower than the video rate: hold the last frame so the video runs in sim time
            t_out += dt
        w.append_data(frame)
        self.t_out[topic], self.prev[topic] = t_out + dt, frame

    def close(self):
        for w in self.writers.values():
            w.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("targets", nargs="+", help="TOPIC=FILE.mp4")
    ap.add_argument("--fps", type=int, default=20)
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(dict(t.split("=", 1) for t in args.targets), args.fps)
    done = []
    signal.signal(signal.SIGTERM, lambda *_: done.append(1))
    signal.signal(signal.SIGINT, lambda *_: done.append(1))
    while not done:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
