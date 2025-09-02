# 自己位置推定とマッピング（Localization & Mapping）

This tutorial introduces **LiDAR-SLAM in Minecraft with ROS 2 (Humble)** to help you experience how a robot estimates its **own position** and builds a **map** of the environment.

---

## What is Localization?

Localization is the technique that allows a robot to estimate **where it is** in the world.

It combines sensor data such as wheel odometry, IMU, LiDAR, and camera, to estimate the robot’s **pose** (position x, y, z and orientation yaw/pitch/roll).

If a **map already exists**, the robot can match its sensor observations against the map to determine its current location.

> Analogy: Like your smartphone maps app showing your current position on the map using GPS and sensors.

---

## What is SLAM?

SLAM (**Simultaneous Localization And Mapping**) is used when **no prior map exists**. It allows the robot to **build a map** and **localize itself within that map** at the same time.

* **Localization**: Needs an existing map, only estimates the robot’s position.
* **SLAM**: Builds the map while simultaneously estimating the position.

In this tutorial, you will run **LiDAR-SLAM** in the Minecraft world.

---

## Tutorial Overview

1. Install ROS 2 Humble and developer tools
2. Fetch and build the sample workspace
3. Run SLAM in one terminal
4. Run the Minecraft client in another terminal
5. Move the robot and observe how `/map` and `/tf` update in real time

---

## Requirements

* **ROS 2 Humble** (Ubuntu 22.04)
* Tools: `git`, `vcstool`, `colcon`, `rosdep`
* Repository: `minecraft_ros2_example`
* Packages: `minecraft_ros2` (via `.repos` file)
* Minecraft client (`runClient.sh` script)

---

## Setup

### 0. Install Development Tools

```bash
sudo apt update
sudo apt install -y git python3-vcstool python3-colcon-common-extensions python3-rosdep
# rosdep initialization (only once)
sudo rosdep init || true
rosdep update
```

### 1. Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

### 2. Fetch the Sample Workspace

Follow the [sample setup guide](/documentation/setup_sample) to fetch and build the example.

---

## How to Run

### Terminal 1: Start SLAM

```bash
source ~/ros2_java_ws/install/setup.bash
source ~/minecraft_ros2_example_ws/install/setup.bash
ros2 launch minecraft_ros2_example lidarslam_ros2.launch.xml
```

* SLAM and bridge nodes will start
* Topics like `/scan`, `/map`, `/tf` will be published

### Terminal 2: Start Minecraft Client

```bash
cd ~/minecraft_ros2
./runClient.sh
```

* Minecraft client starts in ROS-connected mode

---

## How to Verify

### Check Topics

```bash
source ~/minecraft_ros2/install/setup.bash
ros2 topic list
```

### Check Map Updates

```bash
ros2 topic echo /map --qos-durability=transient_local
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
# Open frames.pdf to see map -> odom -> base_link
```

### Visualization (Optional)

```bash
rviz2
```

In RViz, add:

* **Map** (`/map`)
* **LaserScan / PointCloud** (`/scan`)
* **TF**

---

## Troubleshooting

* **`command not found: ros2/colcon`** → Make sure you sourced `/opt/ros/humble/setup.bash`.
* **`rosdep` errors** → Run `sudo rosdep init` and `rosdep update`.
* **Missing or non-executable `runClient.sh`** → `chmod +x runClient.sh`.
* **Build fails (missing deps)** → Run `rosdep install ...` again.
* **No topics appearing** → Ensure Terminal 1 (SLAM) is running and Terminal 2 (client) is connected.

---

## Summary

* **Localization**: Estimate your position on an existing map.
* **SLAM**: Build a map and localize at the same time.
* With this tutorial, you can run **LiDAR-SLAM in Minecraft**, and watch how `/map` and `/tf` update with the robot’s movement.

Try visualizing with RViz for deeper understanding.
