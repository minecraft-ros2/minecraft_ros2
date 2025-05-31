# プレイヤー操作

minecraft_ros2内のプレイヤーはTwistメッセージを与えることで操作することができます。

### ROS 2 Interface

| Type                  | Topic Name           |
| --------------------- | -------------------- |
| `geometry_msgs/Twist` | `/cmd_vel`           |