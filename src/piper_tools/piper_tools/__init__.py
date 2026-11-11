"""The Piper arm's hands-on tools (check the arm, e-stop, start/stop session, poses, gripper) as installable commands.

Pure Python on top of `piper_core` and the vendor SDK (pyAgxArm); no ROS. The same code runs from the repo (`python tools/real_arm/arm_stop.py`,
thin shims) and installed on a box (`piper-estop`, ...). Importing the package sends nothing to the arm.
"""
