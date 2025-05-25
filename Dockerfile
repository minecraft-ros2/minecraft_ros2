FROM ghcr.io/minecraft-ros2/ros2_java:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
SHELL ["/bin/bash", "-c"]
WORKDIR /ws/minecraft_ros2
COPY . .

RUN <<EOF
    apt-get update
    apt-get install -y ros-humble-rviz2
    apt-get clean
    rm -rf /var/lib/apt/lists
EOF

COPY --chmod=755 <<EOF runRviz.sh
    source /opt/ros/humble/setup.bash
    rviz2 -d ./minecraft.rviz
EOF
