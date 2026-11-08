#!/usr/bin/env python3
"""Exit 0 once /clock is advancing, 1 on timeout. Gates ros2_control for simulators that start slowly
(Isaac Sim): a controller manager running on sim time cannot activate controllers before the clock ticks."""

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
    rclpy.init()
    node = Node("wait_for_clock", parameter_overrides=[])
    seen = []
    node.create_subscription(Clock, "/clock", lambda m: seen.append(m.clock.sec + m.clock.nanosec * 1e-9),
                             qos_profile_sensor_data)
    deadline = node.get_clock().now().nanoseconds * 1e-9 + timeout
    while rclpy.ok() and len(seen) < 20 and node.get_clock().now().nanoseconds * 1e-9 < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = len(seen) >= 20 and seen[-1] > seen[0]
    node.get_logger().info("clock is running" if ok else "timed out waiting for /clock")
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
