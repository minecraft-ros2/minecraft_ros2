FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    default-jdk \
    gradle \
    curl \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U git+https://github.com/colcon/colcon-gradle \
    && python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle \
    && rm -rf /root/.cache/pip

WORKDIR /ws/ros2_java_ws

RUN rosdep update
RUN mkdir -p src
RUN curl -skL https://raw.githubusercontent.com/kazu-321/ros2_java/main/ros2_java_desktop.repos | vcs import src
RUN rosdep install --from-paths src -y -i --skip-keys "ament_tools"

ENV ROS2JAVA_INSTALL_PATH=/ws/ros2_java_ws/install

RUN source /opt/ros/humble/setup.bash  && \
    colcon build

WORKDIR /ws/minecraft_ros2
COPY . .

RUN ./gradlew prepareRunClientCompile
# Note: Minecraft client should be started manually as per README instructions
CMD ["./runClient.sh"]