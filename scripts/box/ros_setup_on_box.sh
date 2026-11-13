#!/usr/bin/env bash
# Runs ON THE BOX, optional: a ROS 2 Jazzy workspace of Piper Studio at ~/piper/ws. No sudo. Stops at the first missing piece and says what the user must do. UNTESTED on the box.
set -euo pipefail
src="$HOME/piper/src/piper_studio"; ws="$HOME/piper/ws"
[ -d "$src" ] || { echo "no $src: run install_on_box.sh first"; exit 1; }
[ -f /opt/ros/jazzy/setup.bash ] || { echo "ROS 2 Jazzy is not installed (/opt/ros/jazzy). Needs apt and sudo: ask the user to install ros-jazzy-ros-base and ros-jazzy-desktop deps (docs/ARCHITECTURE.md lists the packages)."; exit 2; }
command -v git >/dev/null && command -v uv >/dev/null || { echo "needs git and uv (~/.local/bin/uv)"; exit 3; }
mkdir -p "$ws"; rsync -a --delete --exclude external/agx_arm_ros --exclude .venv --exclude build --exclude install --exclude log "$src"/ "$ws"/
if [ ! -d "$ws/external/agx_arm_ros/.git" ]; then
  commit=$(cat "$src/.agx_arm_ros_commit")
  git clone https://github.com/agilexrobotics/agx_arm_ros.git "$ws/external/agx_arm_ros" && git -C "$ws/external/agx_arm_ros" checkout -q "$commit"
fi
cd "$ws"; export PYTHONNOUSERSITE=1
[ -d .venv ] || uv venv --system-site-packages
uv pip sync env/requirements.txt --python .venv/bin/python
set +u; source /opt/ros/jazzy/setup.bash; source .venv/bin/activate; set -u
python -m colcon build --symlink-install
echo "done. In every shell: cd $ws && source scripts/env.sh"
