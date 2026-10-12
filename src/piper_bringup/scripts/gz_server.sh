#!/usr/bin/env bash
# Run `gz sim` as a headless server with EGL rendering (needed for camera sensors without a display) and stop it
# cleanly: `gz` is a Ruby launcher that forks the real server, so signal the whole process group.
#   gz_server.sh WORLD.sdf [extra gz sim args]
set -u
setsid gz sim -s -r --headless-rendering -v 2 "$@" &
pid=$!
# Safety net: if this wrapper is killed outright (SIGKILL), the server must not outlive it.
me=$$
setsid bash -c "while kill -0 $me 2>/dev/null; do sleep 1; done; kill -TERM -- -$pid 2>/dev/null; sleep 5; kill -KILL -- -$pid 2>/dev/null" >/dev/null 2>&1 </dev/null &
stop() {
  kill -INT -- -"$pid" 2>/dev/null; kill -TERM -- -"$pid" 2>/dev/null
  for _ in $(seq 1 40); do kill -0 "$pid" 2>/dev/null || exit 0; sleep 0.25; done
  kill -KILL -- -"$pid" 2>/dev/null
  exit 0
}
trap stop INT TERM
wait "$pid"
