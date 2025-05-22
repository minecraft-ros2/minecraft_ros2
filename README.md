# Minecraft\_ros2

[日本語](README_JP.md)

**minecraft_ros2** is a MOD that integrates Minecraft with ROS 2. It allows Minecraft world data—such as player position and block states—to be published to ROS 2, enabling visualization in RViz2 and potential interaction from ROS 2 nodes.

This repository contains the source code for the MOD and provides a foundation for experiments and research involving Minecraft and robotics middleware.

---

## Requirements

* Ubuntu 22.04
* Minecraft (Java Edition)
* ROS 2 Humble or later
* [ros2\_java](https://github.com/kazu-321/ros2_java)

---

## Installation Guide

### 1. Install Minecraft

Install Minecraft Java Edition on Ubuntu 22.04 and verify that it runs correctly.

### 2. Build ros2\_java

Follow the instructions at the following repository to build `ros2_java`:

👉 [https://github.com/kazu-321/ros2\_java](https://github.com/kazu-321/ros2_java)

### 3. Set Environment Variable

Add the following line to your `.bashrc` or shell configuration file to specify the `install` directory of `ros2_java`:

```bash
export ROS2JAVA_INSTALL_PATH=/home/USERNAME/ros2_java_ws/install
```

After editing, apply the change by running:

```bash
source ~/.bashrc
```

### 4. Launch Minecraft

Run the following script included in this repository to start Minecraft with the MOD:

```bash
./runClient.sh
```

### 5. Visualize in RViz2

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
