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

### ROS 2

Type: `sensor_msgs/PointCloud2`
Topic Name: `/player/pointcloud`

## Camera

### ROS 2

Type: `sensor_msgs/Image`
Topic Name: `/player/image_rawd`

## IMU

### ROS 2

Type: `sensor_msgs/Imu`
Topic Name: `/player/imu`
