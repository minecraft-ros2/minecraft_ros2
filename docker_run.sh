xhost +local:root

GPU_OPT=""
if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi

GRADLE_VOLUME="minecraft_ros2_gradle_cache"
docker volume inspect "$GRADLE_VOLUME" >/dev/null 2>&1 || docker volume create "$GRADLE_VOLUME" >/dev/null

docker run -it --rm $GPU_OPT --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --mount source="$GRADLE_VOLUME",target=/root/.gradle minecraft_ros2:latest 
xhost -local:root
