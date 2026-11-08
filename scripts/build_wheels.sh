#!/usr/bin/env bash
# Build the wheels downstream projects install (no ROS needed to install them): piper_core, piper_assets and piper_mujoco.
#   scripts/build_wheels.sh [--allow-dirty]      -> dist/*.whl   (needs the ROS environment once, to generate the assets)
# The version is the one in each package; the source commit is written into the wheels. Refuses a dirty tree: a release comes from a commit.
set -euo pipefail
cd "$(dirname "$0")/.."
set +u; source scripts/env.sh >/dev/null; set -u
if [ "${1:-}" != "--allow-dirty" ] && [ -n "$(git status --porcelain -- src tools scripts)" ]; then
  echo "uncommitted changes in src/tools/scripts: commit first (or --allow-dirty for a test build)" >&2; exit 1
fi
commit=$(git rev-parse HEAD)
rm -rf dist && mkdir dist
python tools/release/build_assets.py
python tools/release/build_mujoco.py
echo "SOURCE_COMMIT = \"$commit\"" > src/piper_core/piper_core/_source.py
for pkg in piper_core piper_assets piper_mujoco; do
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
  python -m pip wheel --no-build-isolation --no-deps -q -w dist "src/$pkg"
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
done
rm -f src/piper_core/piper_core/_source.py
( cd dist && sha256sum *.whl > SHA256SUMS && ls -l )
