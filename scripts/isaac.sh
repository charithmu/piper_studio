#!/usr/bin/env bash
# Isaac Sim helpers for Piper Studio. Isaac runs in its own environment (never installed into .venv).
#   scripts/isaac.sh build-usd     export the description (ROS side) and convert it to USD (Isaac side)
#   scripts/isaac.sh run [args]    run the Isaac runner (isaac/run_piper.py); args: --seconds N --video f.mp4 --gui
#   scripts/isaac.sh check         compare Isaac's USD with the URDF (fingertip pose over random configs, mass)
#   scripts/isaac.sh stop          stop this workspace's runner (by PID file; never touches other Isaac processes)
# Configuration (environment variables):
#   ISAAC_ENV_SH   script that activates an Isaac Sim 6.x python env   (default ~/projects/sim/robosim/env.sh)
#   PIPER_ISAAC_DATA   generated URDF/USD location                     (default ~/data/ml/isaac/piper_studio)
#   ROS_DOMAIN_ID  must equal the domain of the ROS side (default 0)
# See isaac/README.md for what this needs and how to recreate it on another machine.
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ENV_SH="${ISAAC_ENV_SH:-$HOME/projects/sim/robosim/env.sh}"
DATA="${PIPER_ISAAC_DATA:-$HOME/data/ml/isaac/piper_studio}"
PIDFILE="$DATA/run.pid"
USD="$DATA/usd/piper_isaac/piper_isaac.usda"
cmd="${1:-}"; shift || true

# Run a command in a clean environment (no system ROS, no ~/.local) inside Isaac's env, on the GPU.
in_isaac() {
  env -i HOME="$HOME" PATH="/usr/bin:/bin:$HOME/.local/bin" \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" PIPER_ISAAC_PIDFILE="$PIDFILE" \
    ${GZ_PARTITION:+GZ_PARTITION="$GZ_PARTITION"} \
    bash -c 'source "$0"; export LD_LIBRARY_PATH="$ISAAC_SIM_DIR/exts/isaacsim.ros2.core/jazzy/lib"; exec gpu-run "$@"' \
    "$ISAAC_ENV_SH" "$@"
}

case "$cmd" in
  build-usd)
    mkdir -p "$DATA"; rm -rf "${DATA:?}/usd"   # the importer would otherwise write to usd/piper_isaac_1
    ( set +u; source "$ws/scripts/env.sh"
      ros2 run piper_description export_urdf.py --hardware isaac --physics --out "$DATA/piper_isaac.urdf" )
    in_isaac python "$ws/isaac/convert_urdf.py" "$DATA/piper_isaac.urdf" "$DATA/usd" > "$DATA/convert.log" 2>&1
    grep "USD:" "$DATA/convert.log" ;;
  run)
    [ -f "$USD" ] || { echo "no USD at $USD; run: scripts/isaac.sh build-usd" >&2; exit 1; }
    [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null && { echo "runner already running (pid $(cat "$PIDFILE"))" >&2; exit 1; }
    # gpu-run does not forward signals, so forward them to the runner (it writes its pid to $PIDFILE).
    in_isaac python "$ws/isaac/run_piper.py" --usd "$USD" "$@" &
    child=$!
    trap '"$0" stop' INT TERM
    wait "$child" || true
    # a trapped signal interrupts wait; wait again until the runner is really gone
    while kill -0 "$child" 2>/dev/null; do wait "$child" || true; done ;;
  check)
    ( set +u; source "$ws/scripts/env.sh"; python "$ws/tools/make_fk_reference.py" "$DATA/fk_reference.json" )
    in_isaac python "$ws/isaac/check_model.py" "$USD" "$DATA/fk_reference.json" 2>&1 | grep "check_model" ;;
  stop)
    [ -f "$PIDFILE" ] || { echo "not running"; exit 0; }
    pid="$(cat "$PIDFILE")"
    kill -TERM "$pid" 2>/dev/null || { rm -f "$PIDFILE"; exit 0; }
    for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || { echo "stopped"; exit 0; }; sleep 1; done
    kill -KILL "$pid" 2>/dev/null; rm -f "$PIDFILE"; echo "killed" ;;
  *) sed -n 2,9p "$0"; exit 2 ;;
esac
