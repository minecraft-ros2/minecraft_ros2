xhost +local:root

GPU_OPT=""
if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi

GRADLE_VOLUME="minecraft_ros2_gradle_cache"
docker volume inspect "$GRADLE_VOLUME" >/dev/null 2>&1 \
  || docker volume create "$GRADLE_VOLUME" >/dev/null

if [ ! -d ~/.minecraft ]; then
  mkdir ~/.minecraft
  echo "Created ~/.minecraft directory"
fi

docker run -it --rm \
  $GPU_OPT \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --mount source="$GRADLE_VOLUME",target=/root/.gradle \
  -v ~/.minecraft:/ws/minecraft_ros2/run \
  minecraft_ros2:latest \
  /bin/bash ./runClient.sh
xhost -local:root
