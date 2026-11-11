#!/usr/bin/env bash
# The OPTIONAL cuRobo environment: only needed to REGENERATE the cuRobo sphere model (tools/curobo/). Running, controlling or simulating the arm, the wheels, and
# reading the finished model (piper_assets.curobo_model) never need it.
#   scripts/curobo_env.sh setup            create .venv-curobo (cuRobo + torch, about 7 GB) from third_party/curobo at the pinned revision (network once; asks nothing else)
#   scripts/curobo_env.sh status           what exists
#   scripts/curobo_env.sh python [args]    run that interpreter
#   scripts/curobo_env.sh run SCRIPT [args]  run a script in it on the GPU (through gpu-run: reserves the GPU, unloads the LLMs)
# Pins: env/curobo.rev (cuRobo commit), env/constraints-curobo.txt (versions the model was validated with). Own env, not shared (RULES.md section 2).
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; venv="$ws/.venv-curobo"; src="$ws/third_party/curobo"; rev="$(cat "$ws/env/curobo.rev")"
case "${1:-status}" in
  setup)
    if [ ! -d "$src/.git" ]; then
      mkdir -p "$ws/third_party"; git clone https://github.com/NVlabs/curobo.git "$src"
    fi
    git -C "$src" fetch -q origin 2>/dev/null || true
    git -C "$src" checkout -q "$rev"
    [ -x "$venv/bin/python" ] || uv venv --python 3.11 "$venv"
    uv pip install --python "$venv/bin/python" -c "$ws/env/constraints-curobo.txt" -e "$src[cu13-torch]" trimesh scipy pyyaml pytest rtree
    "$0" status ;;
  status)
    [ -x "$venv/bin/python" ] || { echo "not set up: scripts/curobo_env.sh setup"; exit 1; }
    echo "cuRobo checkout: $(git -C "$src" rev-parse --short HEAD) (pinned $(echo "$rev" | cut -c1-7))"
    "$venv/bin/python" -c "import torch, curobo, trimesh, warp; print('torch', torch.__version__, 'cuda', torch.cuda.is_available(), '| warp', warp.__version__, '| trimesh', trimesh.__version__)" ;;
  python) shift; exec "$venv/bin/python" "$@" ;;
  run) shift; exec gpu-run "$venv/bin/python" "$@" ;;
  *) sed -n 2,10p "$0"; exit 2 ;;
esac
