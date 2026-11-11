"""Provisional joint servo gains for Isaac Sim, kept equal to MuJoCo's (piper_sim/config/mujoco/inputs.xml).

(stiffness, damping, max force): N*m/rad, N*m*s/rad, N*m for revolute joints; N/m, N*s/m, N for the gripper.
damping = the kv MuJoCo derives from dampratio and the link inertia; test_servo_gains.py checks the two agree.
These stand in for the unidentified firmware position loop (see docs/description/DECISIONS.md).
"""

# With the ZED X Mini on its bracket (the default): the wrist is heavier, so the kv MuJoCo derives for joints 4-6 is about twice the robot-only one.
GAINS = {
    "joint1": (400, 15.7, 100), "joint2": (400, 16.8, 100), "joint3": (400, 18.0, 100),
    "joint4": (100, 3.3, 100), "joint5": (100, 2.7, 100), "joint6": (50, 0.74, 100),
    "gripper": (400, 8, 10), "gripper_joint1": (400, 8, 10), "gripper_joint2": (400, 8, 10),
}
# The robot only (camera none).
GAINS_NO_CAMERA = {
    "joint1": (400, 15.7, 100), "joint2": (400, 16.4, 100), "joint3": (400, 16.5, 100),
    "joint4": (100, 1.7, 100), "joint5": (100, 2.3, 100), "joint6": (50, 0.31, 100),
    "gripper": (400, 8, 10), "gripper_joint1": (400, 8, 10), "gripper_joint2": (400, 8, 10),
}


def gains(camera: bool = True) -> dict:
    return GAINS if camera else GAINS_NO_CAMERA
