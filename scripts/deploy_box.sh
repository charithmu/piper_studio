#!/usr/bin/env bash
# Build the bundle the ZED Box needs to run the Piper arm tools, and (only with --push) send and install it.
#   scripts/deploy_box.sh                      build box-bundle/ (wheels piper_core, piper_assets, piper_tools; pinned deps; arm config; installer). Touches nothing else.
#   scripts/deploy_box.sh --push [user@host]   also copy it to the box (~/piper-bundle/) and run its install_on_box.sh there (default zeddev@10.50.0.11)
# The workstation does not use this: here the repo's .venv and tools/real_arm/ shims run the same code (docs/real_arm/BOX_DEPLOY.md).
# Pushing writes files and installs packages on the box: do it only with the user's approval (RULES.md section 6). Nothing here moves the arm.
set -euo pipefail
cd "$(dirname "$0")/.."
push=0; target="zeddev@10.50.0.11"
for a in "$@"; do case "$a" in --push) push=1 ;; -h|--help) sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; *@*) target="$a" ;; esac; done
set +u; source scripts/env.sh >/dev/null; set -u
out=box-bundle
commit=$(git rev-parse --short HEAD); dirty=""; [ -n "$(git status --porcelain -- src tools scripts)" ] && dirty="-dirty"
rm -rf "$out" && mkdir -p "$out/arms"
python tools/release/build_assets.py >/dev/null
echo "SOURCE_COMMIT = \"$(git rev-parse HEAD)$dirty\"" > src/piper_core/piper_core/_source.py
for pkg in piper_core piper_assets piper_tools; do
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
  python -m pip wheel --no-build-isolation --no-deps -q -w "$out" "src/$pkg"
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
done
rm -f src/piper_core/piper_core/_source.py
cp docs/real_arm/arms/*.yaml "$out/arms/"
# deps installed on the box from PyPI / GitHub (the box has internet); same pins as env/requirements.in. numpy: the box's system Python has none visible with the user site off.
{ printf '%s\n' "numpy>=1.24" pyyaml python-can; grep -E '^pyAgxArm' env/requirements.in; } > "$out/pins.txt"
echo "piper arm tools ${commit}${dirty} $(date -u +%FT%TZ)" > "$out/VERSION"
cp scripts/box/install_on_box.sh "$out/"; chmod +x "$out/install_on_box.sh"
( cd "$out" && sha256sum *.whl > SHA256SUMS && ls -l )
echo "bundle: $PWD/$out  ($(tr "\n" " " < "$out/pins.txt"))"
if [ "$push" = 1 ]; then
  ssh "$target" 'rm -rf ~/piper-bundle && mkdir -p ~/piper-bundle'
  scp -q -r "$out"/. "$target:piper-bundle/"
  ssh -t "$target" 'cd ~/piper-bundle && ./install_on_box.sh'
else
  echo "not pushed. To install on the box (needs the user's approval):  scripts/deploy_box.sh --push"
fi
