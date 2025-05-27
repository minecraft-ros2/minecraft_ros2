# Source Installation

## System Requirements

* Ubuntu 22.04
* ROS 2 Humble

## Install Build Tools

1. **Install Gradle**

   ```bash
   sudo apt install gradle
   ```

    ::: info
    Make sure you have Gradle 3.2 (or later) installed.
    :::

2. **Install required tools**

   ```bash
   sudo apt install curl python3-colcon-common-extensions python3-pip python3-vcstool
   ```

3. **Install Gradle extensions for colcon**

   ```bash
   python3 -m pip install -U git+https://github.com/colcon/colcon-gradle
   python3 -m pip install --no-deps -U git+https://github.com/colcon/colcon-ros-gradle
   ```


## ros2\_java Setup

1. **Download the ROS 2 Java repositories into a workspace**

   ```bash
   mkdir -p ros2_java_ws/src
   cd ros2_java_ws
   curl -skL https://raw.githubusercontent.com/minecraft-ros2/ros2_java/main/ros2_java_desktop.repos | vcs import src
   ```

2. **Install ROS dependencies**

   ```bash
   rosdep install --from-paths src -y -i --skip-keys "ament_tools"
   ```

3. **Build the desktop packages**

   ```bash
   colcon build
   ```
    ::: warning
    Do not use  `--symlink-install`
    :::


## minecraft\_ros2 Installation

1. **Set the environment variable**
   Add the following line to your shell config file (e.g. `~/.bashrc`), pointing to the `ros2_java` install directory:

   ```bash
   export ROS2JAVA_INSTALL_PATH=/home/USERNAME/ros2_java_ws/install
   ```

    ::: info
    Replace `USERNAME` with the appropriate username for your environment. You can verify it by running the `pwd` command.
    :::

   Then reload your shell:

   ```bash
   source ~/.bashrc
   ```


2. **Launch Minecraft with the mod**

   ```bash
   git clone https://github.com/minecraft-ros2/minecraft_ros2.git
   ./runClient.sh
   ```

3. **Visualize in RViz2**
   Start RViz2 and load the provided configuration to view Minecraft data:

   ```bash
   rviz2 -d minecraft.rviz
   ```
