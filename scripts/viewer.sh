#!/usr/bin/env bash
# run|start|stop|status the Piper Studio model viewer (tools/viewer, loopback port 8781). Stops by PID file only.
#   scripts/viewer.sh start      then open http://localhost:8781/  (tailnet: https://<host>:8700/piper-studio/viewer/ once registered)
cd "$(dirname "$0")/.." || exit 1
mkdir -p tools/viewer/run
pid=tools/viewer/run/server.pid
case "${1:-status}" in
  run)   # foreground, for the systemd user unit (autostart at boot, registered in artifacts.yaml); the unit stops it
    source scripts/env.sh
    exec python tools/viewer/server.py 8781;;
  start)
    if [ -f "$pid" ] && kill -0 "$(cat $pid)" 2>/dev/null; then echo "already running ($(cat $pid))"; exit 0; fi
    # shellcheck disable=SC1091
    source scripts/env.sh
    nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('python',['python','tools/viewer/server.py','8781'])" > tools/viewer/run/server.log 2>&1 &
    echo $! > "$pid"; sleep 2; echo "started $(cat $pid): http://localhost:8781/";;
  stop)
    [ -f "$pid" ] && kill "$(cat $pid)" 2>/dev/null; rm -f "$pid"; echo stopped;;
  status)
    [ -f "$pid" ] && kill -0 "$(cat $pid)" 2>/dev/null && echo "running $(cat $pid)" || echo "not running";;
esac
