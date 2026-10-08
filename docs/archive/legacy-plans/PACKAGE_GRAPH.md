# Piper Studio Package Graph

This diagram is the fastest way to explain how the workspace is organized.
It includes every ROS package currently present under `src/`, grouped by role.

![Piper Studio package graph](PACKAGE_GRAPH.svg)

Solid arrows show current package or architecture dependencies that exist today.
Dashed arrows show launch-time composition or the intended perception-to-manipulation flow described in the workspace design.

## How to read it

- `pyAgxArm` is the authoritative hardware SDK and sits below the ROS driver.
- `agx_arm_ros` contains the upstream-frozen ROS bridge, robot description, messages, and base MoveIt config.
- `agx_arm_gzsim` and `agx_arm_mjsim` are alternative simulation overlays that reuse the same upstream description and MoveIt stack.
- `agx_arm_motion` is the convenience layer for motion primitives, but it is still tied to the Gazebo-backed sim profile today.
- `agx_arm_workspace` is the composition package for Piper-specific real-hardware bringup and workspace URDF assembly.
- `agx_arm_wristcam`, `agx_arm_detect`, `agx_arm_graspgen`, and `agx_arm_manipulation` form the planned perception-to-action pipeline.
- `moveit_calibration_plugins` and `moveit_calibration_gui` are vendored support packages for calibration workflows rather than Piper-specific application packages.

## Mermaid Source

```mermaid
flowchart LR
  classDef upstream fill:#e7f0ff,stroke:#245ea8,color:#0f2747,stroke-width:1px;
  classDef workspace fill:#eaf8ee,stroke:#2f7d49,color:#153822,stroke-width:1px;
  classDef planned fill:#fff5e6,stroke:#b36b00,color:#5a3500,stroke-width:1px;
  classDef support fill:#f3edf9,stroke:#6b4aa5,color:#2f1a4f,stroke-width:1px;

  subgraph sdk[Standalone SDK]
    pyagxarm["pyAgxArm<br/>CAN SDK<br/>standalone, not built with colcon"]
  end

  subgraph upstream_stack[Upstream-frozen core stack]
    msgs["agx_arm_msgs<br/>custom ROS interfaces"]
    ctrl["agx_arm_ctrl<br/>hardware bridge and bringup"]
    desc["agx_arm_description<br/>URDF and Xacro robot models"]
    moveit["agx_arm_moveit<br/>base MoveIt configuration"]
  end

  subgraph workspace_stack[Workspace-owned simulation and workspace packages]
    gzsim["agx_arm_gzsim<br/>Gazebo Harmonic overlay"]
    mjsim["agx_arm_mjsim<br/>MuJoCo overlay"]
    motion["agx_arm_motion<br/>MoveIt facade<br/>currently sim-first"]
    ws["agx_arm_workspace<br/>workspace URDF and real-arm bringup"]
  end

  subgraph perception_stack[Workspace-owned perception and manipulation]
    wristcam["agx_arm_wristcam<br/>camera adapter and calibration"]
    detect["agx_arm_detect<br/>AprilTag and detection pipelines"]
    graspgen["agx_arm_graspgen<br/>grasp candidate generation"]
    manipulation["agx_arm_manipulation<br/>task-level behaviors"]
  end

  subgraph support_stack[Vendored support packages]
    calib_plugins["moveit_calibration_plugins<br/>calibration back-end plugins"]
    calib_gui["moveit_calibration_gui<br/>RViz calibration UI"]
  end

  pyagxarm -->|authoritative low-level SDK| ctrl
  msgs --> ctrl
  desc --> moveit
  desc --> gzsim
  desc --> mjsim
  moveit --> gzsim
  moveit --> mjsim
  moveit --> motion
  gzsim -->|current sim profile dependency| motion

  desc -.->|composes robot model| ws
  ctrl -.->|wraps hardware bringup| ws
  moveit -.->|wraps real-arm MoveIt launch| ws
  wristcam -.->|composes selected camera into workspace model| ws

  calib_plugins --> calib_gui
  calib_gui -.->|supports eye-in-hand calibration workflows| wristcam

  wristcam -.->|standardized camera topics and TF| detect
  detect -.->|tag poses and TF| graspgen
  graspgen -.->|grasp candidates| manipulation
  motion -.->|motion primitives| manipulation

  class pyagxarm,msgs,ctrl,desc,moveit upstream;
  class gzsim,mjsim,motion,ws workspace;
  class wristcam,detect,graspgen,manipulation planned;
  class calib_plugins,calib_gui support;
```