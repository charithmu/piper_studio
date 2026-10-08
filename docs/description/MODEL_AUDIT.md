# Piper description audit

Generated 2026-10-08 by `src/piper_description/tools/model_audit.py` (MuJoCo 3.12.0). Conclusions are in [DECISIONS.md](DECISIONS.md).

Reference: `official@983788b`. Values differing from the reference are **bold**. FK compares `link6` relative to `base_link`.

| Source | Kind | Origin |
|---|---|---|
| official@983788b | urdf | agilexrobotics/agx_arm_urdf@983788b `piper/urdf/piper_with_gripper_description.xacro` |
| official@3080af4(arm) | urdf | agilexrobotics/agx_arm_urdf@3080af4 `piper/urdf/piper_description.urdf` (previous pin, arm only) |
| isaac@8e1f88f | urdf | agilexrobotics/piper_isaac_sim@8e1f88f `piper_description/urdf/piper_description.urdf` (source of the official USDs) |
| menagerie@feadf76 | mjcf (mujoco 3.12.0) | google-deepmind/mujoco_menagerie@feadf76 `agilex_piper/piper.xml` |
| legacy_mjsim@95f6d89 | mjcf (mujoco 3.12.0) | charithmu/agx_arm_mjsim@95f6d89 `mjcf/piper.xml` (legacy Piper Studio MuJoCo) |

## Arm joint limits (rad, rad/s, N·m)

| Joint | official@983788b | official@3080af4(arm) | isaac@8e1f88f | menagerie@feadf76 | legacy_mjsim@95f6d89 |
|---|---|---|---|---|---|
| joint1 lower | -2.618 | -2.618 | -2.618 | -2.618 | -2.618 |
| joint1 upper | 2.618 | 2.618 | **2.168** | 2.618 | 2.618 |
| joint1 vel | 5 | 5 | 5 | — | — |
| joint1 effort | 100 | 100 | 100 | 100 | 100 |
| joint2 lower | 0 | 0 | 0 | 0 | 0 |
| joint2 upper | 3.1416 | 3.1416 | **3.14** | **3.14** | **3.14** |
| joint2 vel | 5 | 5 | 5 | — | — |
| joint2 effort | 100 | 100 | 100 | 100 | 100 |
| joint3 lower | -2.9671 | -2.9671 | -2.967 | **-2.697** | **-2.697** |
| joint3 upper | 0 | 0 | 0 | 0 | 0 |
| joint3 vel | 5 | 5 | 5 | — | — |
| joint3 effort | 100 | 100 | 100 | 100 | 100 |
| joint4 lower | -1.7453 | -1.7453 | -1.745 | **-1.832** | **-1.832** |
| joint4 upper | 1.7453 | 1.7453 | 1.745 | **1.832** | **1.832** |
| joint4 vel | 5 | 5 | 5 | — | — |
| joint4 effort | 100 | 100 | 100 | 100 | 100 |
| joint5 lower | -1.2217 | -1.2217 | **-1.22** | **-1.22** | **-1.22** |
| joint5 upper | 1.2217 | 1.2217 | **1.22** | **1.22** | **1.22** |
| joint5 vel | 5 | 5 | 5 | — | — |
| joint5 effort | 100 | 100 | 100 | 100 | 100 |
| joint6 lower | -2.0944 | -2.0944 | -2.0944 | **-3.14** | **-3.14** |
| joint6 upper | 2.0944 | 2.0944 | 2.0944 | **3.14** | **3.14** |
| joint6 vel | 5 | 5 | **3** | — | — |
| joint6 effort | 100 | 100 | 100 | 100 | 100 |

## Gripper joints

| Source | Joints (type, range, mimic) |
|---|---|
| official@983788b | `gripper` prismatic [0, 0.1]; `gripper_joint1` prismatic [0, 0.05] mimic `gripper`; `gripper_joint2` prismatic [-0.05, 0] mimic `gripper` |
| official@3080af4(arm) |  |
| isaac@8e1f88f | `joint7` prismatic [0, 0.035]; `joint8` prismatic [-0.035, 0] |
| menagerie@feadf76 | `joint7` prismatic [0, 0.035]; `joint8` prismatic [-0.035, 0] |
| legacy_mjsim@95f6d89 | `gripper_joint1` prismatic [0, 0.035]; `gripper_joint2` prismatic [-0.035, 0] |

## Masses (kg)

| Link | official@983788b | official@3080af4(arm) | isaac@8e1f88f | menagerie@feadf76 | legacy_mjsim@95f6d89 |
|---|---|---|---|---|---|
| base_link | 1.02 | 1.02 | 1.02 | **0.1625** | **0.1625** |
| link1 | 0.71 | 0.71 | 0.71 | **0.0979** | **0.0979** |
| link2 | 1.16 | 1.16 | **1.17** | **0.2909** | **0.2909** |
| link3 | 0.5 | 0.5 | 0.5 | **0.2908** | **0.2908** |
| link4 | 0.38 | 0.38 | 0.38 | **0.1271** | **0.1271** |
| link5 | 0.383 | 0.383 | 0.383 | **0.1447** | **0.1447** |
| link6 | 0.0061 | **0.007** | **0.007** | **1.2** | **1.2** |
| flange_link | 0.04 | — | — | — | — |
| gripper_base | 0.45 | — | 0.45 | — | — |
| gripper_link1 | 0.025 | — | — | — | — |
| gripper_link2 | 0.025 | — | — | — | — |
| link7 | — | — | 0.025 | 0.0265 | 0.0265 |
| link8 | — | — | 0.025 | 0.0265 | 0.0265 |
| **total** | 4.6991 | **4.16** | **4.67** | **2.3669** | **2.3669** |

## Forward kinematics of `link6` vs reference

200 random configurations inside the reference arm limits (seed 0), plus the zero pose.

| Source | zero-pose pos err (mm) | max pos err (mm) | max rot err (deg) |
|---|---|---|---|
| official@3080af4(arm) | 0.000 | 0.000 | 0.000 |
| isaac@8e1f88f | 0.016 | 0.050 | 0.004 |
| menagerie@feadf76 | 9.940 | 11.315 | 0.002 |
| legacy_mjsim@95f6d89 | 9.940 | 11.315 | 0.002 |
