#!/usr/bin/env bash
# WSL2 (Ubuntu 24.04) setup for the spider hexapod ROS2 project.
#
# Idempotent: можно перезапускать, шаги, которые уже выполнены, будут пропущены.
# Запускать ИЗНУТРИ WSL: bash /mnt/c/Users/user/vs_code_projects/ogonek-spider/ros2/scripts/setup-wsl.sh
set -euo pipefail

PROJECT_PARENT="$HOME/projects/ogonek-spider"
ROS2_DIR="$PROJECT_PARENT/ros2"
LIBS_DIR="$PROJECT_PARENT/libs/lib"
REPO_URL="https://github.com/ogonek-spider/ros2.git"
TWAI_URL="https://github.com/ogonek-spider/twaiproto.git"

log() { printf '\n\033[1;32m==>\033[0m %s\n' "$*"; }

log "1/6  apt: системные зависимости"
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
    ca-certificates curl git build-essential pkg-config \
    libgl1 libglu1-mesa libxrandr2 libxinerama1 libxcursor1 libxi6 \
    libxext6 libx11-6 libxkbcommon0 libegl1 \
    locales

# UTF-8 локаль (нужна ROS2)
sudo locale-gen en_US.UTF-8 >/dev/null
echo 'export LANG=en_US.UTF-8' | sudo tee /etc/profile.d/locale-utf8.sh >/dev/null

log "2/6  pixi"
if ! command -v pixi >/dev/null 2>&1; then
    curl -fsSL https://pixi.sh/install.sh | bash
fi
# pixi кладёт себя в ~/.pixi/bin
export PATH="$HOME/.pixi/bin:$PATH"
if ! grep -q '.pixi/bin' "$HOME/.bashrc"; then
    echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> "$HOME/.bashrc"
fi
pixi --version

log "3/6  git: SSH→HTTPS rewrite (для submodules без ключей)"
git config --global url."https://github.com/".insteadOf "git@github.com:"
git config --global init.defaultBranch master

log "4/6  clone репозитория в $ROS2_DIR"
mkdir -p "$PROJECT_PARENT"
if [ ! -d "$ROS2_DIR/.git" ]; then
    git clone --recurse-submodules "$REPO_URL" "$ROS2_DIR"
else
    git -C "$ROS2_DIR" pull --ff-only
    git -C "$ROS2_DIR" submodule update --init --recursive
fi

log "5/6  twai_proto в $LIBS_DIR/twai_proto"
mkdir -p "$LIBS_DIR"
if [ ! -d "$LIBS_DIR/twai_proto/.git" ]; then
    git clone "$TWAI_URL" "$LIBS_DIR/twai_proto"
else
    git -C "$LIBS_DIR/twai_proto" pull --ff-only || true
fi
# В репе twaiproto исходники лежат в lib/twai_proto/, а spider_ros_control
# CMakeLists ждёт их прямо в корне twai_proto/. Симлинки решают.
ln -sf lib/twai_proto/twai_proto.cpp "$LIBS_DIR/twai_proto/twai_proto.cpp"
ln -sf lib/twai_proto/twai_proto.h   "$LIBS_DIR/twai_proto/twai_proto.h"

log "6/6  pixi env (kilted) — это долго, идёт скачивание ROS2 Kilted"
cd "$ROS2_DIR/pixi-robostack"
pixi install -e kilted

cat <<EOF

\033[1;36m=== Готово ===\033[0m

Дальше — собрать и запустить (в WSL):

  cd $ROS2_DIR/pixi-robostack
  pixi shell -e kilted
  cd ..
  colcon build --symlink-install
  source install/local_setup.bash
  ros2 launch spider_ros_control spider-mujoco.launch.py

EOF
