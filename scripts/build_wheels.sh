#!/usr/bin/env bash
# Build the wheels downstream projects install (no ROS needed to install them): piper_core, piper_assets, piper_mujoco, piper_isaac and piper_tools (the box bundle is `scripts/deploy_box.sh`).
#   scripts/build_wheels.sh [--allow-dirty]      -> dist/*.whl   (needs the ROS environment once, to generate the assets)
#   scripts/build_wheels.sh --dev                -> dist-dev/*.whl  a snapshot of the working tree for other projects to try before a release
#                                                   (same version numbers; install with `uv pip install --no-deps --reinstall dist-dev/*.whl`)
# The version is the one in each package; the source commit is written into the wheels (a dev build adds "-dirty" when the tree is not clean).
# A release build refuses a dirty tree: a release comes from a commit.
set -euo pipefail
cd "$(dirname "$0")/.."
set +u; source scripts/env.sh >/dev/null; set -u
out=dist
[ "${1:-}" = "--dev" ] && out=dist-dev
if [ "${1:-}" != "--allow-dirty" ] && [ "${1:-}" != "--dev" ] && [ -n "$(git status --porcelain -- src tools scripts)" ]; then
  echo "uncommitted changes in src/tools/scripts: commit first (or --dev for a snapshot, --allow-dirty for a test build)" >&2; exit 1
fi
commit=$(git rev-parse HEAD)
[ -n "$(git status --porcelain -- src tools scripts)" ] && commit="$commit-dirty"
rm -rf "$out" && mkdir "$out"
python tools/release/build_assets.py
python tools/release/build_mujoco.py
python tools/release/build_isaac.py     # needs the USD of `scripts/isaac.sh build-usd` (and `scripts/isaac.sh check` to have passed)
echo "SOURCE_COMMIT = \"$commit\"" > src/piper_core/piper_core/_source.py
for pkg in piper_core piper_assets piper_mujoco piper_isaac piper_tools; do
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
  python -m pip wheel --no-build-isolation --no-deps -q -w "$out" "src/$pkg"
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
done
rm -f src/piper_core/piper_core/_source.py
( cd "$out" && sha256sum *.whl > SHA256SUMS && ls -l )
