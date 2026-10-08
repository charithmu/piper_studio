#!/usr/bin/env bash
# Rebuild the workspace from a clean clone. Installs only inside the workspace;
# missing system (apt) packages are reported with the exact command to run.
set -euo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws"

[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy not found at /opt/ros/jazzy" >&2; exit 1; }
command -v uv >/dev/null || { echo "uv not found (https://docs.astral.sh/uv/)" >&2; exit 1; }

echo "== sources"
git submodule update --init --recursive

echo "== python environment (.venv, layered on ROS system packages)"
[ -d .venv ] || uv venv --system-site-packages --python /usr/bin/python3.12 .venv
touch .venv/COLCON_IGNORE
uv pip sync --python .venv/bin/python env/requirements.txt

set +u
source /opt/ros/jazzy/setup.bash
set -u
echo "== system dependencies (rosdep)"
if ! rosdep check --from-paths src external --ignore-src -r --skip-keys warehouse_ros_mongo >/dev/null 2>&1; then
  echo "Missing system packages. Review, then run:"
  echo "  sudo apt-get update && rosdep install --from-paths src external --ignore-src -r -y --skip-keys warehouse_ros_mongo"
  rosdep check --from-paths src external --ignore-src -r --skip-keys warehouse_ros_mongo 2>&1 | grep -E "^apt|^ERROR" || true
fi

echo "== build"
set +u
source .venv/bin/activate
set -u
export PYTHONNOUSERSITE=1
python -m colcon build --symlink-install
echo "Done. Use: source scripts/env.sh"
