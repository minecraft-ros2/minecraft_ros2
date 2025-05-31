# センサー

ROS2に対応した多くのセンサーが用意されています。

## LiDAR

### 使用方法

「minecraft\_ros2」で提供されているLiDARセンサーは、防具のヘルメットとして実装されています。クリエイティブモードでは、「戦闘」タブの一番最後に追加されており、プレイヤーがこれを装備することで、点群データのパブリッシュが自動的に開始されます。

![イベントリにLiDARがある](/images/lidar_2.png)
![LiDARを装備](/images/lidar_1.png)

### 対応しているLiDAR
- Velodyne VLP-16
- HESAI XT32
- HESAI FT120
- RS LiDAR M1
- UTM 30LN

### ROS 2 Interface

| Type                      | Topic Name           |
| ------------------------- | -------------------- |
| `sensor_msgs/PointCloud2` | `/player/pointcloud` |

## カメラ

### ROS 2 Interface

| Type                | Topic Name          |
| ------------------- | ------------------- |
| `sensor_msgs/Image` | `/player/image_raw` |

## IMU

### ROS 2 Interface

| Type              | Topic Name    |
| ----------------- | ------------- |
| `sensor_msgs/Imu` | `/player/imu` |