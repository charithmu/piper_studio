# Reference repositories (`for_reference/`)

About 5.3 GB of cloned vendor and community repositories kept next to the code so they can be read and compared without a network round trip.
They are **not part of the build**, are git-ignored, and are all clean clones at the commits we studied.

- List with sizes and what each is for: `scripts/reference_repos.sh`
- Restore one or all (pinned commits): `scripts/reference_repos.sh NAME...` or `scripts/reference_repos.sh all`
- Used by this repo's docs and decisions: `agx_arm_sim` (camera stand and pose, D11), `piper_isaac_sim` (older description, MODEL_AUDIT, the wrong camera rotation), `realsense-ros`, `depthai-ros`, `OrbbecSDK_ROS2` (camera options), `moveit_calibration` and `handeye_calibration_ros` (camera calibration, CLEANUP C9), `QuestArmTeleop`, `lerobot-agilex` (roadmap M6/M7).

**To do at some point: prune.** Roughly 2.5 GB is study material nothing refers to (`Agilex-College` 1 GB, `gr00t-agilex`, `dual_piper`, `piper_ros` (ROS 1), `agilex-arm-gravity-compensation`, `aloha-agilex`, `rdt-agilex`, `openpi-agilex`, and others). Delete a directory when you no longer need to read it; the script restores it. Decided 2026-11-08 to keep everything for now (CLEANUP F1).
