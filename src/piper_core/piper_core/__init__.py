"""Piper arm rules without ROS.

The single implementation of the rules every interface applies to commands for the arm, so the ROS path (`command_guard`), the
direct path and any downstream planner agree:

    from piper_core import GuardConfig, check_command, limits_from_urdf, named_poses_from_srdf, RealArm   # RealArm: the real arm on the vendor SDK

Nothing here imports ROS; the inputs are plain text (a URDF, an SRDF) and numbers.
"""

from piper_core.guard import GuardConfig, Verdict, check_command
from piper_core.limits import limits_from_urdf
from piper_core.poses import classify_state, named_poses_from_srdf, zero_pose
from piper_core.real_arm import RealArm, RealArmError

__all__ = ["GuardConfig", "Verdict", "check_command", "limits_from_urdf", "named_poses_from_srdf",
           "classify_state", "zero_pose", "RealArm", "RealArmError"]
