xhost +local:root
docker run -it --rm --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" minecraft_ros2:latest 
xhost -local:root