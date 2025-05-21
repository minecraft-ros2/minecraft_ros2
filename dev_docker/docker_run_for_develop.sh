#!/usr/bin/env bash
set -e

# X11 の権限を緩める
xhost +local:root

# スクリプトのあるディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

# GPU が使えるなら --gpus オプションを付与
if type nvidia-container-runtime &>/dev/null; then
  GPU_OPT="--gpus all"
else
  GPU_OPT=""
fi

docker run -it --rm $GPU_OPT --net host \
  --privileged \
  -e DISPLAY="$DISPLAY" \
  -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --workdir="/ws/minecraft_ros2" \
  -v "$SCRIPT_DIR/..:/ws/minecraft_ros2" \
  -v "$SCRIPT_DIR/gradle:/root/.gradle" \
  minecraft_ros2_dev:latest

