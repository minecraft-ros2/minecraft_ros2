FROM ghcr.io/minecraft-ros2/ros2_java:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
SHELL ["/bin/bash", "-c"]
RUN apt update \
    && apt install -y -qq collada-urdf-tools blender \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    trimesh \
    numpy==1.26.4 \
    scipy

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
