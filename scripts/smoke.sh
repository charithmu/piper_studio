#!/usr/bin/env bash
# One-command regression check of everything Piper Studio ships. Read-only: starts nothing that stays, moves no robot.
#   scripts/smoke.sh            ~30 s   offline tests (incl. generated macros, wheel data, cuRobo model current), registry, live pages, downstream study (CPU)
#   scripts/smoke.sh --sims     ~2 min  + launch tests on mock and MuJoCo and the carrier example
#   scripts/smoke.sh --all      ~5 min  + Gazebo and Isaac Sim launch tests (GPU through gpu-run) and the study's GPU tests
# Exit status is non-zero if anything failed. Each launch test uses its own ROS domain (scripts/test_all.sh).
set -uo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$ws"
mode="${1:-}"; fail=0
ok() { echo "PASS  $*"; }; bad() { echo "FAIL  $*"; fail=1; }

# 0. orphaned Fast DDS shared memory of crashed runs can make a launch test hang (seen: Isaac, 2026-11-11): warn, do not delete
orph=$(scripts/clean_shm.sh | sed -n 's/.*orphaned: \([0-9]*\).*/\1/p'); [ "${orph:-0}" -gt 300 ] && echo "warn  $orph orphaned Fast DDS shm files in /dev/shm: scripts/clean_shm.sh --yes (nothing a live process holds is touched)"
# 1. offline tests, generated-macro check, and the launch tests asked for
case "$mode" in --sims) sims="mock mujoco" ;; --all) sims="mock gazebo mujoco isaac" ;; *) sims="" ;; esac
if [ "$mode" = --all ]; then gpu-run scripts/test_all.sh $sims; else scripts/test_all.sh $sims; fi; [ $? -eq 0 ] || fail=1

# 2. the artifact registry: nothing declared that is not published, nothing published that is not declared (this project)
out=$(python3 ~/projects/platform/registry/registry.py check --project piper_studio 2>&1)
if echo "$out" | grep -q "^FAIL"; then bad "registry"; echo "$out" | grep "^FAIL"; else ok "registry   $(echo "$out" | head -1)"; fi

# 3. the pages: always-on ones must answer; on-demand ones are only reported
python3 - <<'PY' || fail=1
import sys, urllib.request, yaml
arts = yaml.safe_load(open("artifacts.yaml"))["artifacts"]; bad = 0
for a in arts:
    if a.get("type") != "web" or not a.get("port"): continue
    try: code = urllib.request.urlopen(f"http://127.0.0.1:{a['port']}/", timeout=3).status
    except Exception as e: code = type(e).__name__
    up = code == 200
    state = "PASS" if up else ("FAIL" if a.get("autostart") else "info")
    bad += state == "FAIL"
    print(f"{state:5s} page {a['id']:22s} :{a['port']}  {'running' if up else 'not running (' + str(code) + ')' + ('' if a.get('autostart') else ', on demand: ' + str(a.get('start')))}")
sys.exit(1 if bad else 0)
PY

# 4. the downstream study uses the installed wheel: same version as this tree, and its CPU tests pass
S=~/projects/ml_tests/experiments/curobo_piper_study
clean() { env -i HOME="$HOME" PATH="$(dirname "$(command -v gpu-run)"):/usr/local/bin:/usr/bin:/bin" "$@"; }   # the study has its own venv: the ROS environment must not leak into it
if [ -x "$S/.venv/bin/pytest" ]; then
  want=$(grep -m1 'version = ' src/piper_assets/pyproject.toml | cut -d'"' -f2); have=$(cd /tmp && clean "$S/.venv/bin/python" -c "import piper_assets as p; print(p.__version__)")
  [ "$want" = "$have" ] && ok "study venv has piper_assets $have" || bad "study venv has piper_assets $have, this tree is $want (reinstall the wheels: its docs/07_SETUP.md)"
  (cd "$S" && clean .venv/bin/pytest -q 2>&1 | tail -1 | grep -q passed) && ok "study CPU tests" || bad "study CPU tests"
  if [ "$mode" = --all ]; then (cd "$S" && clean gpu-run .venv/bin/pytest -m gpu -q 2>&1 | tail -1 | grep -q passed) && ok "study GPU tests (cuRobo plans executed on MuJoCo, both arms)" || bad "study GPU tests"; rm -f "$S/ucode.elf"; fi
fi
rm -f ucode.elf
[ $fail -eq 0 ] && echo "ALL OK" || echo "SOMETHING FAILED"; exit $fail
