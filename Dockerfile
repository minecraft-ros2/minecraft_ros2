FROM ghcr.io/minecraft-ros2/ros2_java:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
SHELL ["/bin/bash", "-c"]
RUN apt update \
    && apt install -y -qq collada-urdf-tools blender \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/minecraft_ros2
COPY . .

RUN ./gradlew prepareRunClientCompile
# Note: Minecraft client should be started manually as per README instructions
CMD ["./runClient.sh"]
