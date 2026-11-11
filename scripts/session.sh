# Source this in every terminal you use for Piper Studio work (instead of scripts/env.sh alone):
#   source scripts/session.sh [ROS_DOMAIN_ID]        # default: $ROS_DOMAIN_ID, else 101
#
# It does what each terminal needs:
#   * scripts/env.sh                  ROS Jazzy + .venv + install/ overlay
#   * ROS_DOMAIN_ID                   same value in every terminal of one simulation (Piper range 70-161)
#   * GZ_PARTITION                    what piper.launch.py uses for Gazebo, so `gz sim -g` joins the right simulation
#   * DISPLAY / XAUTHORITY            the TurboVNC desktop (noVNC at https://atlas.buri-fence.ts.net:8443), so windows
#                                     open there even from an SSH terminal
#   * gl <cmd>                        run a GUI program with GPU OpenGL in that desktop (VirtualGL over EGL, no sudo):
#                                     the VNC server itself has no GLX, so RViz/Gazebo/MuJoCo need this prefix
# Override the display with PIPER_VNC_DISPLAY=:N before sourcing. On other machines (ZED Box) DISPLAY is left alone and `gl` runs the command as it is.

if [ -z "${BASH_VERSION:-}" ] || [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "source this file: source scripts/session.sh [ROS_DOMAIN_ID]" >&2; return 2 2>/dev/null || exit 2
fi

_piper_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_piper_ws/scripts/env.sh"

export ROS_DOMAIN_ID="${1:-${ROS_DOMAIN_ID:-101}}"
if [ "$ROS_DOMAIN_ID" -lt 70 ] 2>/dev/null || [ "$ROS_DOMAIN_ID" -gt 161 ] 2>/dev/null; then
  echo "warning: ROS_DOMAIN_ID=$ROS_DOMAIN_ID is outside the Piper range 70-161 (RULES.md); other projects use 0, 1, 7, 42" >&2
fi
export GZ_PARTITION="${GZ_PARTITION:-piper_d$ROS_DOMAIN_ID}"

# Atlas has the TurboVNC desktop on :1; any other machine (the ZED Box, a laptop) keeps its own DISPLAY (headless: none) unless PIPER_VNC_DISPLAY is set.
if [ -n "${PIPER_VNC_DISPLAY:-}" ] || [ "$(hostname -s)" = atlas ]; then
  export DISPLAY="${PIPER_VNC_DISPLAY:-:1}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
fi

if command -v vglrun >/dev/null 2>&1; then gl() { vglrun -d egl "$@"; }; else gl() { "$@"; }; fi

_piper_display=ok
if [ -z "${DISPLAY:-}" ]; then _piper_display="none (headless)"
elif command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then _piper_display="NOT REACHABLE (is the VNC session running?)"; fi
echo "piper session: ROS_DOMAIN_ID=$ROS_DOMAIN_ID  GZ_PARTITION=$GZ_PARTITION  DISPLAY=$DISPLAY ($_piper_display)"
[ -f "$_piper_ws/install/setup.bash" ] || echo "note: workspace not built yet: python -m colcon build --symlink-install" >&2
unset _piper_ws _piper_display
