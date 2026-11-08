#!/usr/bin/env bash
# Run the repo's checks. Offline tests always; launch tests per backend on request.
#   scripts/test_all.sh                      offline tests + generated-macro check (fast, no simulator)
#   scripts/test_all.sh mock gazebo mujoco   also the launch tests of those backends, the carrier example and the real-arm plumbing
#   scripts/test_all.sh isaac                add isaac (GPU; needs `scripts/isaac.sh build-usd d435` first)
# Each launch test gets its own ROS_DOMAIN_ID (from 150 up) and its daemon is stopped afterwards. Exit status is non-zero if anything failed.
set -uo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u; source "$ws/scripts/env.sh"; set -u
cd "$ws"
fail=0; domain=150
run() {  # NAME CMD...
  local name=$1; shift
  if out=$("$@" 2>&1); then echo "PASS  $name   $(echo "$out" | tail -1)"; else echo "FAIL  $name"; echo "$out" | tail -15; fail=1; fi
}
run "offline tests" python -m pytest src/piper_description src/piper_control src/piper_core src/piper_sim src/piper_assets src/piper_mujoco src/piper_isaac -q
run "generated macros vs official" python src/piper_description/tools/piper_arm_macro.py check
for backend in "$@"; do
  domain=$((domain + 1)); export ROS_DOMAIN_ID=$domain
  PIPER_TEST_BACKENDS=$backend run "launch test: $backend" timeout 600 python -m pytest src/piper_py/test/test_bringup.py -q
  ros2 daemon stop >/dev/null 2>&1
done
if [ $# -gt 0 ]; then
  domain=$((domain + 1)); export ROS_DOMAIN_ID=$domain
  run "real-arm plumbing (fake driver)" timeout 600 python -m pytest src/piper_py/test/test_real_plumbing.py src/piper_py/test/test_real_start_stop.py -q
  ros2 daemon stop >/dev/null 2>&1
  domain=$((domain + 1)); export ROS_DOMAIN_ID=$domain
  run "carrier example" timeout 600 python -m pytest src/piper_examples -q
  ros2 daemon stop >/dev/null 2>&1
fi
exit $fail
