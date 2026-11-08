# Masses of the Piper description

Every mass in the model, with where it comes from. **None of these are measured on the real arm**: the arm's firmware does not report masses, the vendor values are
CAD estimates (they changed for link6 on 2026-10-16), and the camera and stand values are placeholders. Do not use them as ratings. Weighing the arm and the camera
parts would replace the entries below.

Regenerate after any description change: `python src/piper_description/tools/mass_table.py` (camera `d435` on the default mount). Vendor source: `agx_arm_urdf` @ 680fa92.

| Link | Mass (kg) | Source | Note |
|---|---:|---|---|
| `base_link` | 1.02 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `link1` | 0.71 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `link2` | 1.16 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `link3` | 0.5 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `link4` | 0.38 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `link5` | 0.383 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `link6` | 0.016554 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `flange_link` | 0.04 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `gripper_base` | 0.45 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `gripper_link` | 0 | vendor URDF (agx_arm_urdf, CAD estimate) | virtual width joint, no mass |
| `gripper_link1` | 0.025 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `gripper_link2` | 0.025 | vendor URDF (agx_arm_urdf, CAD estimate) |  |
| `camera_mount_link` | 0.04 | our placeholder (mounts/agilex_mid_stand.xacro), not weighed |  |
| `camera_link` | 0.072 | cameras/d435.xacro (the file itself says the value is not reliable), not weighed |  |
| **arm (base to link6)** | **4.1696** | sum | |
| **flange + gripper** | **0.5400** | sum | |
| **camera + stand** | **0.1120** | sum | |
| **total with gripper and camera** | **4.8216** | sum | |
| **total arm + gripper (no camera)** | **4.7096** | sum | |
