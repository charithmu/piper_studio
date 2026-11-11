#!/usr/bin/env bash
# start|stop|status the cuRobo model viewer (tools/curobo_viewer, loopback port 8782). Stops by PID file only. No cuRobo needed: it reads the models shipped in piper_assets.
#   scripts/curobo_viewer.sh start      then http://localhost:8782/  (tailnet: https://<host>:8700/piper-studio/curobo/ once registered)
cd "$(dirname "$0")/.." || exit 1
mkdir -p tools/curobo_viewer/run
pid=tools/curobo_viewer/run/server.pid
case "${1:-status}" in
  start)
    if [ -f "$pid" ] && kill -0 "$(cat $pid)" 2>/dev/null; then echo "already running ($(cat $pid))"; exit 0; fi
    # shellcheck disable=SC1091
    source scripts/env.sh
    nohup python3 -c "import os,signal; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp('python',['python','tools/curobo_viewer/server.py','8782'])" > tools/curobo_viewer/run/server.log 2>&1 &
    echo $! > "$pid"; sleep 2; echo "started $(cat $pid): http://localhost:8782/";;
  stop)
    [ -f "$pid" ] && kill "$(cat $pid)" 2>/dev/null; rm -f "$pid"; echo stopped;;
  status)
    [ -f "$pid" ] && kill -0 "$(cat $pid)" 2>/dev/null && echo "running $(cat $pid)" || echo "not running";;
esac
