#!/usr/bin/env python3
"""Executable wrapper for piper_control.real_driver (the ROS driver for the real Piper on piper_core.RealArm)."""

import rclpy
from rclpy.executors import MultiThreadedExecutor

from piper_core.real_arm import RealArmError
from piper_control.real_driver import RealDriver


def main():
    rclpy.init()
    try:
        node = RealDriver()
    except RealArmError as e:
        print(f"piper_real_driver: {e}\nNothing was sent to the arm.")
        rclpy.try_shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
