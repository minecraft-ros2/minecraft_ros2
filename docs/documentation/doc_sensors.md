# Sensors

A variety of sensors compatible with ROS 2 are available.

## LiDAR

### Usage

The LiDAR sensor provided in “minecraft\_ros2” is implemented as a helmet in the armor slot. In Creative Mode, it can be found at the very end of the "Combat" tab. When a player equips it, point cloud data is automatically published.

![LiDAR in inventory](/images/lidar_2.png)
![Equipped LiDAR](/images/lidar_1.png)

### Supported LiDAR Models

* Velodyne VLP-16
* HESAI XT32
* HESAI FT120
* RS LiDAR M1
* UTM 30LN

### ROS 2 Interface

| Type                      | Topic Name           |
| ------------------------- | -------------------- |
| `sensor_msgs/PointCloud2` | `/player/pointcloud` |

## Camera

### ROS 2 Interface

| Type                | Topic Name          |
| ------------------- | ------------------- |
| `sensor_msgs/Image` | `/player/image_raw` |

## IMU

### ROS 2 Interface

| Type              | Topic Name    |
| ----------------- | ------------- |
| `sensor_msgs/Imu` | `/player/imu` |
