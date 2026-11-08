#!/usr/bin/env bash
# Clone the reference repositories (vendor and community code kept for comparison; NOT part of the build, git-ignored) at the commits we studied.
#   scripts/reference_repos.sh            list: name, whether present, size, what it is for
#   scripts/reference_repos.sh NAME...    clone the missing ones (or "all")
# Everything under for_reference/ can be deleted and restored with this script. See docs/REFERENCE_REPOS.md.
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ref="$ws/for_reference"

# name | path under for_reference | url | commit | used for
REPOS=(
"agx_arm_sim|agilex/agx_arm_sim|https://github.com/agilexrobotics/agx_arm_sim.git|f8cd8b1|SOURCE of the camera stand mesh and mount pose (DECISIONS D11)"
"piper_isaac_sim|agilex/piper_isaac_sim|https://github.com/agilexrobotics/piper_isaac_sim.git|8e1f88f|older AgileX Isaac description (the wrongly rotated camera pose came from here); MODEL_AUDIT reference"
"agx_arm_urdf|agilex/agx_arm_urdf|https://github.com/agilexrobotics/agx_arm_urdf.git|680fa92|official URDF (also inside the external/agx_arm_ros submodule)"
"agx_arm_ros|agilex/agx_arm_ros|https://github.com/agilexrobotics/agx_arm_ros.git|286633e|official ROS stack (the submodule is the pinned copy)"
"pyAgxArm|agilex/pyAgxArm|https://github.com/agilexrobotics/pyAgxArm.git|d90e234|the SDK pinned in env/requirements.in"
"realsense-ros|realsense-ros|https://github.com/realsenseai/realsense-ros.git|6d87b071|RealSense driver and description (real camera, M8)"
"depthai-ros|depthai-ros|https://github.com/luxonis/depthai-ros.git|d31a196|OAK-D Pro driver and description (planned camera)"
"OrbbecSDK_ROS2|OrbbecSDK_ROS2|https://github.com/orbbec/OrbbecSDK_ROS2.git|c780bc11|Orbbec camera driver (camera option)"
"moveit_calibration|moveit_calibration|git@github.com:moveit/moveit_calibration.git|3f9d48e|hand-eye calibration (camera mount calibration, CLEANUP C9)"
"handeye_calibration_ros|agilex/handeye_calibration_ros|https://github.com/agilexrobotics/handeye_calibration_ros.git|3f8e296|AgileX hand-eye calibration"
"QuestArmTeleop|agilex/QuestArmTeleop|https://github.com/agilexrobotics/QuestArmTeleop.git|145b803|teleoperation (roadmap M6)"
"lerobot-agilex|agilex/lerobot-agilex|https://github.com/agilexrobotics/lerobot-agilex.git|8469f3a|LeRobot plugin (roadmap M7)"
"data_tools|agilex/data_tools|https://github.com/agilexrobotics/data_tools.git|58d0ec7|AgileX data recording"
"piper_sdk|agilex/piper_sdk|https://github.com/agilexrobotics/piper_sdk.git|b847e53|older SDK, for comparison"
"piper_sdk_demo|agilex/piper_sdk_demo|https://github.com/agilexrobotics/piper_sdk_demo.git|d457fdf|SDK examples"
"PyAgxArm_demo|agilex/PyAgxArm_demo|https://github.com/agilexrobotics/PyAgxArm_demo.git|d72c390|SDK examples"
"Piper_sdk_ui|agilex/Piper_sdk_ui|https://github.com/agilexrobotics/Piper_sdk_ui.git|958f855|SDK GUI"
"agx_arm_kinematics|agilex/agx_arm_kinematics|https://github.com/agilexrobotics/agx_arm_kinematics.git|ed0a90c|kinematics library"
"robotic_arm_kinematics|agilex/robotic_arm_kinematics|https://github.com/agilexrobotics/robotic_arm_kinematics.git|9452e0b|kinematics library"
"agilex_arm_mujoco_sim|agilex/agilex_arm_mujoco_sim|https://github.com/agilexrobotics/agilex_arm_mujoco_sim.git|848e71f|older MuJoCo model (identical to Menagerie, DECISIONS D8)"
"agilex-arm-gravity-compensation|agilex/agilex-arm-gravity-compensation|https://github.com/agilexrobotics/agilex-arm-gravity-compensation.git|8e2545c|gravity compensation study"
"pose_control|agilex/pose_control|https://github.com/agilexrobotics/pose_control.git|f9acb0b|pose control study"
"piper_ros|agilex/piper_ros|https://github.com/agilexrobotics/piper_ros.git|ac41fcb|ROS 1 (noetic) stack, legacy"
"dual_piper|agilex/dual_piper|https://github.com/agilexrobotics/dual_piper.git|3c9c0ba|two-arm setup study"
"aloha-agilex|agilex/aloha-agilex|https://github.com/agilexrobotics/aloha-agilex.git|275cbe9|policy study (ALOHA)"
"gr00t-agilex|agilex/gr00t-agilex|https://github.com/agilexrobotics/gr00t-agilex.git|83ce744|policy study (GR00T)"
"openpi-agilex|agilex/openpi-agilex|https://github.com/agilexrobotics/openpi-agilex.git|432577c|policy study (openpi)"
"rdt-agilex|agilex/rdt-agilex|https://github.com/agilexrobotics/rdt-agilex.git|974e4a8|policy study (RDT)"
"Agilex-College|agilex/Agilex-College|https://github.com/agilexrobotics/Agilex-College.git|c2688be|tutorials and demos (1 GB)"
"Agilex_Pipers|agilex/Agilex_Pipers|https://github.com/agilexrobotics/Agilex_Pipers.git|0b15281|Piper index repository"
)

if [ $# -eq 0 ]; then
  printf '%-34s %-8s %-6s %s\n' NAME PRESENT SIZE "USED FOR"
  for r in "${REPOS[@]}"; do IFS='|' read -r name path url commit use <<<"$r"
    if [ -d "$ref/$path/.git" ]; then p=yes; s=$(du -sh "$ref/$path" | cut -f1); else p=no; s=-; fi
    printf '%-34s %-8s %-6s %s\n' "$name" "$p" "$s" "$use"
  done; exit 0
fi
for want in "$@"; do
  for r in "${REPOS[@]}"; do IFS='|' read -r name path url commit use <<<"$r"
    [ "$want" = all ] || [ "$want" = "$name" ] || continue
    if [ -d "$ref/$path/.git" ]; then echo "present: $name"; continue; fi
    echo "cloning $name @ $commit"; mkdir -p "$(dirname "$ref/$path")"
    git clone --quiet "$url" "$ref/$path" && git -C "$ref/$path" checkout --quiet "$commit"
  done
done
