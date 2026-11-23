#!/usr/bin/env bash
# Put the hand-eye calibration onto the ZED box as a self-contained folder ~/projects/piper-calib (wheels built from this commit, uv project, board, wrapper, instructions).
# Touches nothing else on the box: not ~/piper (the running box server, Studio's venv), not vision-env. Installs nothing (the uv environment is built on the box by `uv lock && uv sync`).
#   scripts/deploy_calibration.sh           builds calib-bundle/ and prints what --push would do
#   scripts/deploy_calibration.sh --push    copies it to the box (the owner runs this; RULES.md section 6)
# Env: PIPER_BOX (ssh alias, default zedbox), BOARD_DIR (where the printed board's JSON is, default ~/data/ml/piper_pick/real).
set -euo pipefail
case "${1:-}" in ""|--push) ;; *) echo "unknown argument: ${1} (only --push is accepted; nothing was done)" >&2; exit 2 ;; esac
cd "$(dirname "$0")/.."
BOX="${PIPER_BOX:-zedbox}"
DEST='~/projects/piper-calib'
BOARD="${BOARD_DIR:-$HOME/data/ml/piper_pick/real}"
OUT=calib-bundle
set +u; source scripts/env.sh >/dev/null; set -u
VERSION=$(python -c "import re,pathlib;print(re.search(r'version = \"([^\"]+)\"', pathlib.Path('src/piper_calibration/pyproject.toml').read_text()).group(1))")
rm -rf "$OUT"; mkdir -p "$OUT/wheels" "$OUT/board" "$OUT/bin" "$OUT/docs"
for pkg in piper_core piper_assets piper_calibration; do
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
  python -m pip wheel --no-build-isolation --no-deps -q -w "$OUT/wheels" "src/$pkg"
  rm -rf "src/$pkg/build" "src/$pkg"/*.egg-info
done
sed "s/@VERSION@/$VERSION/g" src/piper_calibration/box/pyproject.toml > "$OUT/pyproject.toml"
cp src/piper_calibration/box/README.md "$OUT/README.md"
cp docs/CALIBRATION.md "$OUT/docs/CALIBRATION.md"
cp docs/real_arm/BOX_AGENT_CALIBRATION.md "$OUT/docs/AGENT_INSTRUCTIONS.md"
for f in charuco_6x8_30mm.pdf charuco_6x8_30mm.png charuco_6x8_30mm.json arucogrid_6x7_25mm.json; do
  [ -f "$BOARD/$f" ] && cp "$BOARD/$f" "$OUT/board/" || echo "note: $BOARD/$f not found (print and JSON of the board come from there; \`piper-calibration-board\` makes the 6x8 sheet)" >&2
done
cat > "$OUT/bin/piper-calibrate" <<'W'
#!/usr/bin/env bash
# Hand-eye calibration of the wrist camera: runs in the uv environment of ~/projects/piper-calib, never in the system Python or Studio's venv.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
exec "$HOME/.local/bin/uv" run --project "$HERE" piper-calibrate "$@"
W
chmod +x "$OUT/bin/piper-calibrate"
(cd "$OUT" && find . -type f ! -name SHA256SUMS | sort | xargs sha256sum > SHA256SUMS)
echo "built $OUT (version $VERSION, commit $(git rev-parse --short HEAD)):"; (cd "$OUT" && find . -type f | sort)
if [ "${1:-}" != "--push" ]; then
  echo; echo "dry: --push would run:"
  echo "  ssh $BOX 'mkdir -p $DEST'  and  rsync -a --delete (excluding .venv and uv.lock) $OUT/ $BOX:$DEST/   then sha256sum -c on the box"
  exit 0
fi
ssh -o BatchMode=yes "$BOX" "mkdir -p $DEST"
rsync -a --delete --exclude .venv --exclude uv.lock -e "ssh -o BatchMode=yes" "$OUT/" "$BOX:projects/piper-calib/"
ssh -o BatchMode=yes "$BOX" "cd $DEST && sha256sum -c SHA256SUMS --quiet && echo 'checksums ok' && ls"
