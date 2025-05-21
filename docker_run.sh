xhost +local:root
if type nvidia-container-runtime >/dev/null 2>&1; then
  GPU_OPT="--gpus all"
fi
docker run -it --rm ${GPU_OPT} --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" minecraft_ros2:latest 
xhost -local:root
