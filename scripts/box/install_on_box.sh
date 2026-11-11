#!/usr/bin/env bash
# Runs ON THE BOX (the bundle's copy of this file is what deploy_box.sh sends). Installs the Piper arm tools; sends nothing to the arm.
#   ./install_on_box.sh            install or upgrade from the wheels next to this file
# What it touches (all under the user's home, no sudo): ~/piper/ (venv, bin wrappers, env.sh, bundle copy), ~/.local/bin/piper-* (links to the wrappers),
# ~/.config/piper/arms/ (what is mounted on the arm). It never brings CAN up (that needs sudo: see the end) and never starts anything.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$HOME/piper"
venv="$root/venv"
mkdir -p "$root/bin" "$HOME/.local/bin" "$HOME/.config/piper/arms"

# system site-packages so pyzed (ZED SDK) stays visible; PYTHONNOUSERSITE keeps ~/.local's NumPy 2 out of the way
[ -x "$venv/bin/python" ] || python3 -m venv --system-site-packages "$venv"
export PYTHONNOUSERSITE=1
"$venv/bin/python" -m pip install --quiet --upgrade pip
"$venv/bin/python" -m pip install --quiet -r "$here/pins.txt"
"$venv/bin/python" -m pip install --quiet --no-deps --force-reinstall "$here"/piper_core-*.whl "$here"/piper_assets-*.whl "$here"/piper_tools-*.whl

cp "$here"/arms/*.yaml "$HOME/.config/piper/arms/"
cp "$here"/pins.txt "$here"/SHA256SUMS "$here"/VERSION "$root/" 2>/dev/null || true

# wrappers: every command runs the venv's script with the user site switched off, and finds the data and config folders
for s in "$venv"/bin/piper-*; do
  n="$(basename "$s")"
  cat > "$root/bin/$n" <<WRAP
#!/usr/bin/env bash
export PYTHONNOUSERSITE=1
exec "$venv/bin/$n" "\$@"
WRAP
  chmod +x "$root/bin/$n"
  ln -sf "$root/bin/$n" "$HOME/.local/bin/$n"
done
cat > "$root/env.sh" <<ENV
# source ~/piper/env.sh : the piper commands on PATH and the venv's Python as python (for piper_box and other scripts)
export PYTHONNOUSERSITE=1
export PATH="$root/bin:\$PATH"
ENV

echo "installed: $(cat "$here/VERSION" 2>/dev/null)"
echo "commands:  $(ls "$root/bin" | tr '\n' ' ')"
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) echo "note: ~/.local/bin is not on this shell's PATH (a login shell adds it); use: source ~/piper/env.sh" ;; esac
echo
echo "check (read-only):"
"$root/bin/piper-estop" --help >/dev/null && echo "  piper-estop starts"
PYTHONNOUSERSITE=1 "$venv/bin/python" - <<'PY'
import piper_core, piper_assets, piper_tools, pyAgxArm, can
print("  imports ok:", piper_core.__file__.split("/venv/")[0] + "/venv", "| can", can.__version__)
try:
    import pyzed.sl as sl; print("  pyzed", sl.Camera.get_sdk_version())
except Exception as e:
    print("  pyzed NOT visible:", e)
PY
echo
echo "next, by hand (sudo asks for your password):  piper-can-up      # finds the USB-CAN adapter by USB (can0/can1 swap between boots), 1 Mbit/s; the SoC's own CAN is never touched"
echo "then, with nothing moving:  piper-info --arm-id piper-01   and   piper-joints --once"
echo "emergency stop, typed in a second terminal and not run until needed:  piper-estop"
