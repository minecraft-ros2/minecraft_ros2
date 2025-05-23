#!/usr/bin/env bash
set -euo pipefail

xhost +local:root

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

if type nvidia-container-runtime &>/dev/null; then
  GPU_OPT="--gpus all"
else
  GPU_OPT=""
fi

GRADLE_VOLUME="minecraft_ros2_gradle_cache"

docker volume inspect "$GRADLE_VOLUME" >/dev/null 2>&1 || \
  docker volume create "$GRADLE_VOLUME" >/dev/null

if [ ! -d ~/.minecraft ]; then
  mkdir ~/.minecraft
  echo "Created ~/.minecraft directory"
fi


docker run -it --rm $GPU_OPT --net host \
  -e DISPLAY="$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --workdir="/ws/minecraft_ros2_dev" \
  -v "$SCRIPT_DIR:/ws/minecraft_ros2_dev" \
  --mount source="$GRADLE_VOLUME",target=/root/.gradle -v ~/.minecraft:/ws/minecraft_ros2/run \
  minecraft_ros2:latest /bin/bash

xhost -local:root
