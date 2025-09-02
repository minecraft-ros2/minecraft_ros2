# Sample Code Setup

## 1. Download

Set **`~/minecraft_ros2_example_ws` as the root of the workspace**, as shown below:

```bash
mkdir -p ~/minecraft_ros2_example_ws/src
cd ~/minecraft_ros2_example_ws
# Clone the example repository
git clone https://github.com/minecraft-ros2/minecraft_ros2_example.git
# Use .repos to fetch all dependency packages into src/
vcs import src < minecraft_ros2_example/example.repos
```

## 2. Resolve Dependencies

```bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

## 3. Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
