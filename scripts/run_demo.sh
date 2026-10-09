#!/usr/bin/env bash
# Run the scripted demo (piper_py.demo) on one backend and store the record.
#   scripts/run_demo.sh BACKEND OUTDIR [ROS_DOMAIN_ID] [MODE]    BACKEND: mock | gazebo | mujoco | isaac; MODE: demo (default) | stepresp
# isaac also records the Isaac camera view to OUTDIR/isaac.mp4. Uses the GPU only for isaac.
set -uo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
backend="${1:?backend}"; out="${2:?outdir}"; mkdir -p "$out"
export ROS_DOMAIN_ID="${3:-$((140 + RANDOM % 60))}"
set +u; source "$ws/scripts/env.sh"; set -u
extra=(); [ "$backend" = isaac ] && extra=("isaac_video:=$out/isaac.mp4")
setsid ros2 launch piper_bringup piper.launch.py "backend:=$backend" "${extra[@]}" > "$out/$backend.launch.log" 2>&1 < /dev/null &
lpid=$!
trap 'kill -INT -- -"$lpid" 2>/dev/null; sleep 8; kill -KILL -- -"$lpid" 2>/dev/null' EXIT
for _ in $(seq 1 120); do grep -q "You can start planning" "$out/$backend.launch.log" 2>/dev/null && break; sleep 2; done
mode="${4:-demo}"; file="$out/$backend.json"; [ "$mode" = stepresp ] && file="$out/$backend.step.json"
ros2 run piper_py piper "$mode" --backend "$backend" --out "$file" | tee "$out/$backend.$mode.log"
exit "${PIPESTATUS[0]}"
