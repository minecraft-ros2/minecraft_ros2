FROM ghcr.io/minecraft-ros2/ros2_java:latest

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
SHELL ["/bin/bash", "-c"]

WORKDIR /ws/minecraft_ros2
COPY . .
