#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)

command_name="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

backend="sim"
arm_type="piper"
effector_type="agx_gripper"
revo2_type="left"
namespace=""
can_port="can0"
tcp_offset="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
speed_percent="100"
auto_enable="true"
fast_mode="false"
pub_rate="200"
enable_timeout="5.0"
follow="true"
control="false"
use_rviz="true"
gui="true"
gz_args=""
log_level="info"
gripper_default_effort="1.0"
frame_id="base_link"
x="0.25"
y="0.0"
z="0.30"
roll="0.0"
pitch="1.5708"
yaw="0.0"
can_name="can0"
bitrate="1000000"
usb_address=""
dry_run="false"

usage() {
  cat <<'EOF'
Usage:
  scripts/piper_studio.sh <command> [options]

Commands:
  build           Build the workspace, optionally with --packages pkg1,pkg2
  model-viz       Plain RViz model visualization via agx_arm_description
  driver          Real arm driver only via agx_arm_ctrl
  viz             Visualization workflow with --backend sim|real
  moveit          Planning workflow with --backend sim|real
  moveit-demo     RViz + MoveIt demo from agx_arm_moveit
  motion-once     One-shot TCP motion goal via agx_arm_motion (sim or real backend)
  motion-server   Pose goal server via agx_arm_motion (sim or real backend)
  setup-can       Run the upstream single-CAN activation helper
  scan-can        Show detected CAN interfaces and USB bus addresses
  help            Show this help

Common options:
  --backend sim|real
  --arm-type TYPE
  --effector-type TYPE
  --revo2-type left|right
  --namespace NAME
  --can-port NAME
  --tcp-offset '[x, y, z, rx, ry, rz]'
  --speed-percent N
  --auto-enable true|false
  --fast-mode true|false
  --pub-rate HZ
  --enable-timeout SEC
  --follow true|false
  --control true|false
  --use-rviz true|false
  --gui true|false
  --gz-args '...'
  --log-level LEVEL
  --dry-run

Motion options:
  --x M --y M --z M --roll RAD --pitch RAD --yaw RAD --frame-id FRAME

CAN helper options:
  --can-name NAME
  --bitrate VALUE
  --usb-address BUS

Examples:
  scripts/piper_studio.sh build --packages agx_arm_motion,agx_arm_gzsim
  scripts/piper_studio.sh model-viz --arm-type piper --effector-type agx_gripper
  scripts/piper_studio.sh driver --can-port can0 --arm-type piper --effector-type agx_gripper
  scripts/piper_studio.sh viz --backend sim
  scripts/piper_studio.sh viz --backend real --can-port can0 --arm-type piper --effector-type agx_gripper
  scripts/piper_studio.sh moveit --backend sim
  scripts/piper_studio.sh moveit --backend real --can-port can0 --arm-type nero --effector-type revo2 --revo2-type left
  scripts/piper_studio.sh moveit-demo --arm-type piper_x --effector-type none
  scripts/piper_studio.sh motion-once --x 0.30 --z 0.25 --pitch 1.5708
  scripts/piper_studio.sh motion-server
  scripts/piper_studio.sh scan-can
  scripts/piper_studio.sh setup-can --can-name can0 --bitrate 1000000

Notes:
  - The script activates .venv and sources install/setup.bash when available.
  - motion-once and motion-server support --backend sim|real.  Both require a
    running move_group: start it first with 'moveit --backend sim|real'.
  - The sim viz and sim MoveIt commands target the Piper + gripper Gazebo stack.
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

workspace_source() {
  if [[ -f "$WORKSPACE_ROOT/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE_ROOT/.venv/bin/activate"
  fi

  if [[ -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE_ROOT/install/setup.bash"
  fi
}

run_cmd() {
  local -a cmd=("$@")
  printf 'Running:'
  printf ' %q' "${cmd[@]}"
  printf '\n'

  if [[ "$dry_run" == "true" ]]; then
    return 0
  fi

  workspace_source
  (
    cd "$WORKSPACE_ROOT"
    "${cmd[@]}"
  )
}

parse_args() {
  packages_csv=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --backend) backend="$2"; shift 2 ;;
      --arm-type) arm_type="$2"; shift 2 ;;
      --effector-type) effector_type="$2"; shift 2 ;;
      --revo2-type) revo2_type="$2"; shift 2 ;;
      --namespace) namespace="$2"; shift 2 ;;
      --can-port) can_port="$2"; shift 2 ;;
      --tcp-offset) tcp_offset="$2"; shift 2 ;;
      --speed-percent) speed_percent="$2"; shift 2 ;;
      --auto-enable) auto_enable="$2"; shift 2 ;;
      --fast-mode) fast_mode="$2"; shift 2 ;;
      --pub-rate) pub_rate="$2"; shift 2 ;;
      --enable-timeout) enable_timeout="$2"; shift 2 ;;
      --follow) follow="$2"; shift 2 ;;
      --control) control="$2"; shift 2 ;;
      --use-rviz) use_rviz="$2"; shift 2 ;;
      --gui) gui="$2"; shift 2 ;;
      --gz-args) gz_args="$2"; shift 2 ;;
      --log-level) log_level="$2"; shift 2 ;;
      --gripper-default-effort) gripper_default_effort="$2"; shift 2 ;;
      --frame-id) frame_id="$2"; shift 2 ;;
      --x) x="$2"; shift 2 ;;
      --y) y="$2"; shift 2 ;;
      --z) z="$2"; shift 2 ;;
      --roll) roll="$2"; shift 2 ;;
      --pitch) pitch="$2"; shift 2 ;;
      --yaw) yaw="$2"; shift 2 ;;
      --packages) packages_csv="$2"; shift 2 ;;
      --can-name) can_name="$2"; shift 2 ;;
      --bitrate) bitrate="$2"; shift 2 ;;
      --usb-address) usb_address="$2"; shift 2 ;;
      --dry-run) dry_run="true"; shift ;;
      --help|-h) usage; exit 0 ;;
      *) die "Unknown option: $1" ;;
    esac
  done
}

append_if_set() {
  local array_name="$1"
  local key="$2"
  local value="$3"
  if [[ -n "$value" ]]; then
    eval "$array_name+=(\"${key}:=${value}\")"
  fi
}

parse_args "$@"

case "$command_name" in
  help)
    usage
    ;;

  build)
    if [[ -n "$packages_csv" ]]; then
      IFS=',' read -r -a packages <<< "$packages_csv"
      run_cmd colcon build --packages-select "${packages[@]}"
    else
      run_cmd colcon build
    fi
    ;;

  driver)
    launch_args=(
      "log_level:=$log_level"
      "namespace:=$namespace"
      "can_port:=$can_port"
      "arm_type:=$arm_type"
      "effector_type:=$effector_type"
      "revo2_type:=$revo2_type"
      "auto_enable:=$auto_enable"
      "fast_mode:=$fast_mode"
      "speed_percent:=$speed_percent"
      "pub_rate:=$pub_rate"
      "enable_timeout:=$enable_timeout"
      "tcp_offset:=$tcp_offset"
      "gripper_default_effort:=$gripper_default_effort"
    )
    run_cmd ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py "${launch_args[@]}"
    ;;

  model-viz)
    launch_args=(
      "arm_type:=$arm_type"
      "effector_type:=$effector_type"
      "revo2_type:=$revo2_type"
      "namespace:=$namespace"
      "gui:=$gui"
      "follow:=false"
      "control:=$control"
      "pub_rate:=$pub_rate"
      "tcp_offset:=$tcp_offset"
    )
    run_cmd ros2 launch agx_arm_description display.launch.py "${launch_args[@]}"
    ;;

  viz)
    if [[ "$backend" == "sim" ]]; then
      launch_args=("use_rviz:=$use_rviz")
      append_if_set launch_args "gz_args" "$gz_args"
      run_cmd ros2 launch agx_arm_gzsim piper_with_gripper_gzsim.launch.py "${launch_args[@]}"
    elif [[ "$backend" == "real" ]]; then
      launch_args=(
        "log_level:=$log_level"
        "namespace:=$namespace"
        "can_port:=$can_port"
        "arm_type:=$arm_type"
        "effector_type:=$effector_type"
        "revo2_type:=$revo2_type"
        "auto_enable:=$auto_enable"
        "fast_mode:=$fast_mode"
        "speed_percent:=$speed_percent"
        "pub_rate:=$pub_rate"
        "enable_timeout:=$enable_timeout"
        "follow:=$follow"
        "control:=$control"
        "tcp_offset:=$tcp_offset"
        "gripper_default_effort:=$gripper_default_effort"
      )
      run_cmd ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py "${launch_args[@]}"
    else
      die "Unsupported backend for viz: $backend"
    fi
    ;;

  moveit)
    if [[ "$backend" == "sim" ]]; then
      run_cmd ros2 launch agx_arm_gzsim piper_with_gripper_moveit_gzsim.launch.py
    elif [[ "$backend" == "real" ]]; then
      launch_args=(
        "log_level:=$log_level"
        "namespace:=$namespace"
        "can_port:=$can_port"
        "arm_type:=$arm_type"
        "effector_type:=$effector_type"
        "revo2_type:=$revo2_type"
        "auto_enable:=$auto_enable"
        "fast_mode:=$fast_mode"
        "speed_percent:=$speed_percent"
        "pub_rate:=$pub_rate"
        "enable_timeout:=$enable_timeout"
        "follow:=$follow"
        "tcp_offset:=$tcp_offset"
        "gripper_default_effort:=$gripper_default_effort"
      )
      run_cmd ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py "${launch_args[@]}"
    else
      die "Unsupported backend for moveit: $backend"
    fi
    ;;

  moveit-demo)
    launch_args=(
      "arm_type:=$arm_type"
      "effector_type:=$effector_type"
      "revo2_type:=$revo2_type"
      "namespace:=$namespace"
      "follow:=$follow"
      "tcp_offset:=$tcp_offset"
      "use_rviz:=$use_rviz"
    )
    run_cmd ros2 launch agx_arm_moveit demo.launch.py "${launch_args[@]}"
    ;;

  motion-once)
    launch_args=(
      "backend:=$backend"
      "x:=$x"
      "y:=$y"
      "z:=$z"
      "roll:=$roll"
      "pitch:=$pitch"
      "yaw:=$yaw"
      "frame_id:=$frame_id"
    )
    run_cmd ros2 launch agx_arm_motion move_to_pose.launch.py "${launch_args[@]}"
    ;;

  motion-server)
    launch_args=(
      "backend:=$backend"
    )
    run_cmd ros2 launch agx_arm_motion pose_goal_server.launch.py "${launch_args[@]}"
    ;;

  setup-can)
    helper_args=("$can_name" "$bitrate")
    if [[ -n "$usb_address" ]]; then
      helper_args+=("$usb_address")
    fi
    run_cmd bash "$WORKSPACE_ROOT/src/agx_arm_ros/scripts/can_activate.sh" "${helper_args[@]}"
    ;;

  scan-can)
    run_cmd bash "$WORKSPACE_ROOT/src/agx_arm_ros/scripts/find_all_can_port.sh"
    ;;

  *)
    usage
    die "Unknown command: $command_name"
    ;;
esac