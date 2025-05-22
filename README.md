# Minecraft\_ros2
[![build mod](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/build_test.yaml/badge.svg)](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/build_test.yaml)
[![build docker image](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/minecraft-ros2/minecraft_ros2/actions/workflows/docker_build.yaml)

[日本語](README_JP.md)

**minecraft_ros2** is a MOD that integrates Minecraft with ROS 2. It allows Minecraft world data—such as player position and block states—to be published to ROS 2, enabling visualization in RViz2 and potential interaction from ROS 2 nodes.

This repository contains the source code for the MOD and provides a foundation for experiments and research involving Minecraft and robotics middleware.

---

## Requirements

* Ubuntu 22.04
* Minecraft (Java Edition)
* ROS 2 Humble or later
* [ros2\_java](https://github.com/minecraft-ros2/ros2_java)

---
## 🚢 Quick Start with Docker

Docker provides a convenient way to try out the environment in this repository. Follow the steps below to get started.

> **⚠️ Prerequisites**
> Docker must be installed, and you need to configure your host to share GUI applications (e.g., using `xhost`).

### Steps

1. **Allow GUI Access**

   ```bash
   xhost +local:root
   ```

2. **Start the Docker Container**

   ```bash
   docker run -it --rm \
     --env="DISPLAY=$DISPLAY" \
     --env="QT_X11_NO_MITSHM=1" \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     ghcr.io/minecraft-ros2/minecraft_ros2:latest
   ```

   * If you want to use the GPU, add `--gpus all` (requires NVIDIA Container Toolkit):

3. **Verify Operation with Tools like `rviz2`**

   Once ROS 2 is running inside the container, you can start tools like `rviz2` from another terminal to visualize the data.
---
## Source Installation Guide

### 1. Build ros2\_java

Follow the instructions at the following repository to build `ros2_java`:

👉 [https://github.com/minecraft-ros2/ros2\_java](https://github.com/minecraft-ros2/ros2_java)

### 2. Set Environment Variable

Add the following line to your `.bashrc` or shell configuration file to specify the `install` directory of `ros2_java`:

```bash
export ROS2JAVA_INSTALL_PATH=/home/USERNAME/ros2_java_ws/install
```

After editing, apply the change by running:

```bash
source ~/.bashrc
```

### 3. Launch Minecraft

Run the following script included in this repository to start Minecraft with the MOD:

```bash
./runClient.sh
```

### 4. Visualize in RViz2

Load `minecraft.rviz` in RViz2 to visualize Minecraft data:

```bash
rviz2 -d minecraft.rviz
```

---

## License

This project is licensed under the GNU Lesser General Public License v2.1 (LGPL-2.1).

---

## Contributing

Contributions are welcome! Feel free to open issues or pull requests.

---

## Author

* [kazu-321](https://github.com/kazu-321)

---
