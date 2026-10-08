# Source this file: ROS Jazzy, then the workspace venv, then the workspace overlay (if built).
#   source scripts/env.sh
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$_ws/.venv/bin/activate"
# ~/.local holds NumPy 2 and other user packages that break ROS Jazzy's NumPy 1.26 ABI.
export PYTHONNOUSERSITE=1
if [ -f "$_ws/install/setup.bash" ]; then source "$_ws/install/setup.bash"; fi
unset _ws
