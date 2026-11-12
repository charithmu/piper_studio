#!/usr/bin/env bash
# Isaac Sim helpers for Piper Studio. Isaac runs in its own environment (never installed into .venv).
#   scripts/isaac.sh build-usd [no_camera|go2]  export the description (ROS side) and convert it to USD (Isaac side): with the ZED X Mini wrist camera (default), the robot only, or the DOG's visual for the Go2 setup (the arm USD is the default one)
#   scripts/isaac.sh run [args]    run the Isaac runner (isaac/run_piper.py); args: --seconds N --video f.mp4 --gui
#   scripts/isaac.sh check-go2     the Go2 setup: the dog's visual sits inside the planner's box and on the floor, and the arm is BLOCKED by the dog's box and the contact is reported (a real Isaac run)
#   scripts/isaac.sh check         compare Isaac's USDs with the URDF (fingertip pose over random configs, mass; the camera USD also the bracket/camera frames and collision meshes)
#   scripts/isaac.sh stop          stop this workspace's runner (by PID file; never touches other Isaac processes)
# Configuration (environment variables):
#   ISAAC_ENV_SH   script that activates an Isaac Sim 6.x python env   (default ~/projects/sim/robosim/env.sh)
#   PIPER_ISAAC_DATA   generated URDF/USD location                     (default ~/data/ml/isaac/piper_studio)
#   ROS_DOMAIN_ID  must equal the domain of the ROS side (default 0)
#   PIPER_ISAAC_FRAME_HZ  app/ROS frame rate of the runner (default 60; physics is 240 Hz)
# See isaac/README.md for what this needs and how to recreate it on another machine.
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ENV_SH="${ISAAC_ENV_SH:-$HOME/projects/sim/robosim/env.sh}"
DATA="${PIPER_ISAAC_DATA:-$HOME/data/ml/isaac/piper_studio}"
PIDFILE="$DATA/run.pid"
USD="$DATA/usd/piper_isaac/piper_isaac.usda"                                          # with the wrist camera (the default)
USD_NOCAM="$DATA/usd_no_camera/piper_isaac_no_camera/piper_isaac_no_camera.usda"       # the robot only
USD_DOG="$DATA/usd_go2_dog/piper_isaac_go2_dog/piper_isaac_go2_dog.usda"               # the dog alone (visual only): the Go2 setup = the default arm USD + this + the planner's boxes as static colliders
cmd="${1:-}"; shift || true

# Run a command in a clean environment (no system ROS, no ~/.local) inside Isaac's env, on the GPU.
in_isaac() {
  env -i HOME="$HOME" PATH="/usr/bin:/bin:$HOME/.local/bin" \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" PIPER_ISAAC_PIDFILE="$PIDFILE" ${PIPER_ISAAC_FRAME_HZ:+PIPER_ISAAC_FRAME_HZ="$PIPER_ISAAC_FRAME_HZ"} \
    ${GZ_PARTITION:+GZ_PARTITION="$GZ_PARTITION"} \
    ${DISPLAY:+DISPLAY="$DISPLAY"} ${XAUTHORITY:+XAUTHORITY="$XAUTHORITY"} \
    bash -c 'source "$0"; export LD_LIBRARY_PATH="$ISAAC_SIM_DIR/exts/isaacsim.ros2.core/jazzy/lib"; exec gpu-run "$@"' \
    "$ISAAC_ENV_SH" "$@"
}

case "$cmd" in
  build-usd)
    mkdir -p "$DATA"
    if [ "${1:-}" = no_camera ]; then
      rm -rf "${DATA:?}/usd_no_camera"   # the importer would otherwise write to a numbered sibling
      ( set +u; source "$ws/scripts/env.sh"
        ros2 run piper_sim export_urdf.py --hardware isaac --physics --camera none --out "$DATA/piper_isaac_no_camera.urdf" )
      in_isaac python "$ws/isaac/convert_urdf.py" "$DATA/piper_isaac_no_camera.urdf" "$DATA/usd_no_camera" > "$DATA/convert_no_camera.log" 2>&1
      grep "USD:" "$DATA/convert_no_camera.log"
    elif [ "${1:-}" = go2 ]; then
      rm -rf "${DATA:?}/usd_go2_dog" "${DATA:?}/usd_go2"
      ( set +u; source "$ws/scripts/env.sh"
        ros2 run piper_sim export_urdf.py --go2-dog --out "$DATA/piper_isaac_go2_dog.urdf" )
      in_isaac python "$ws/isaac/convert_urdf.py" "$DATA/piper_isaac_go2_dog.urdf" "$DATA/usd_go2_dog" > "$DATA/convert_go2_dog.log" 2>&1
      grep "USD:" "$DATA/convert_go2_dog.log"
    else
      rm -rf "${DATA:?}/usd"
      ( set +u; source "$ws/scripts/env.sh"
        ros2 run piper_sim export_urdf.py --hardware isaac --physics --camera zedxm --out "$DATA/piper_isaac.urdf" \
          --camera-pose-out "$DATA/camera_pose.json" )
      in_isaac python "$ws/isaac/convert_urdf.py" "$DATA/piper_isaac.urdf" "$DATA/usd" > "$DATA/convert.log" 2>&1
      grep "USD:" "$DATA/convert.log"
    fi ;;
  run)
    usd="$USD"; extra=(--camera-pose "$DATA/camera_pose.json")      # the wrist camera is on by default
    case " $* " in *" --camera none "*) usd="$USD_NOCAM"; extra=() ;; esac
    [ -f "$usd" ] || { echo "no USD at $usd; run: scripts/isaac.sh build-usd [no_camera]" >&2; exit 1; }
    [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null && { echo "runner already running (pid $(cat "$PIDFILE"))" >&2; exit 1; }
    # gpu-run does not forward signals, so forward them to the runner (it writes its pid to $PIDFILE).
    in_isaac python "$ws/isaac/run_piper.py" --usd "$usd" "${extra[@]}" "$@" &
    child=$!
    trap '"$0" stop' INT TERM
    wait "$child" || true
    # a trapped signal interrupts wait; wait again until the runner is really gone
    while kill -0 "$child" 2>/dev/null; do wait "$child" || true; done ;;
  check)
    ( set +u; source "$ws/scripts/env.sh"; python "$ws/isaac/make_fk_reference.py" "$DATA/fk_reference_cam.json" 10 )
    echo "[check_model] --- with the wrist camera (default) ---"
    in_isaac python "$ws/isaac/check_model.py" "$USD" "$DATA/fk_reference_cam.json" 2>&1 | grep "check_model"
    if [ -f "$USD_NOCAM" ]; then
      ( set +u; source "$ws/scripts/env.sh"; python "$ws/isaac/make_fk_reference.py" "$DATA/fk_reference.json" --no-camera )
      echo "[check_model] --- robot only ---"
      in_isaac python "$ws/isaac/check_model.py" "$USD_NOCAM" "$DATA/fk_reference.json" 2>&1 | grep "check_model"
    fi ;;
  check-go2)
    [ -f "$USD_DOG" ] || { echo "no USD at $USD_DOG; run: scripts/isaac.sh build-usd go2" >&2; exit 1; }
    [ -f "$USD" ] || { echo "no USD at $USD; run: scripts/isaac.sh build-usd" >&2; exit 1; }
    echo "[check_go2] --- placement: the dog's visual inside the planner's box, standing on the ground ---"
    in_isaac python "$ws/isaac/check_go2.py" "$USD" "$USD_DOG" 2>&1 | grep "check_go2"
    echo "[check_go2] --- physics: the dog's box blocks the arm and the contact is reported (GPU tests) ---"
    ( set +u; source "$ws/scripts/env.sh"; cd "$ws"; PIPER_TEST_ISAAC=1 python -m pytest src/piper_isaac/test/test_piper_isaac.py -k go2 -q 2>&1 | tail -3 ) ;;
  stop)
    [ -f "$PIDFILE" ] || { echo "not running"; exit 0; }
    pid="$(cat "$PIDFILE")"
    kill -TERM "$pid" 2>/dev/null || { rm -f "$PIDFILE"; exit 0; }
    for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || { echo "stopped"; exit 0; }; sleep 1; done
    kill -KILL "$pid" 2>/dev/null; rm -f "$PIDFILE"; echo "killed" ;;
  *) sed -n 2,9p "$0"; exit 2 ;;
esac
