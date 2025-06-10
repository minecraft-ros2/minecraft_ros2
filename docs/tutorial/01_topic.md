# topic
A topic is an essential communication method when working with ROS 2.

In this page, we will learn the following topics:
- message
- publisher, subscriber
- `ros2 topic` command
- `ros2 interface` command

## message
The information exchanged via topics is called a message.

Message types can be defined freely.

For example, by using `msg/Bool` from the `std_msgs` package, you can perform topic communication with bool types.

In `minecraft_ros2`, the package `minecraft_msgs` defines its own custom message types.

## publisher, subscriber
The sender is called a publisher, and the receiver is called a subscriber.

In other words, when a publisher sends a message to a topic, any subscriber watching that topic receives it and executes some action.

## `ros2 topic` command
Using the `ros2 topic` command, you can check information related to topic communication.

Here are commonly used subcommands: list, echo, info, pub.

### list
By running `ros2 topic list`, you can check the list of currently existing topics.

Additionally, using the `-v` option allows you to see detailed information.

Try running `minecraft_ros2` and joining a world, and you should see an output like this:

```bash
    $ ros2 topic list
/cmd_vel
/parameter_events
/player/image_raw
/player/imu
/player/nearby_living_entities
/player/pointcloud
/player/status
/rosout
/tf
```

### echo
By running `ros2 topic echo topic_name`, you can check the data flowing through the topic.

With `minecraft_ros2` running, try `ros2 topic echo /player/status` to check the player's status.

```bash
    $ ros2 topic echo /player/status 
name: Dev
dimension: minecraft:overworld
food_level: 20
score: 0
.....
```

### info
By running `ros2 topic info topic_name`, you can see information such as the topic's publishers and subscribers.

```bash
    $ ros2 topic info /player/status 
Type: minecraft_msgs/msg/PlayerStatus
Publisher count: 1
Subscription count: 0
```

### pub
By running `ros2 topic pub topic_name type message`, you can publish a message to the topic.

For example, to send a command to `/cmd_vel` to move forward at 1 m/s and to the left at 0.3 m/s:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.3}}"
```

`geometry_msgs/msg/Twist` is a message type for 3D linear and angular movements. `linear.x` means forward linear movement, and `linear.y` means leftward linear movement.

The command might keep running, so stop it with Ctrl + C and then send an empty message.

If you omit the message, an empty message will be sent.

## `ros2 interface` command
By using the `ros2 interface` command, you can obtain information about messages and more.
Mainly use `package` and `show`.

### package
By running `ros2 interface package package_name`, you can retrieve the messages included in that package.

```bash
    $ ros2 interface package minecraft_msgs 
minecraft_msgs/srv/Command
minecraft_msgs/msg/BlockArray
minecraft_msgs/msg/LivingEntity
minecraft_msgs/msg/LivingEntityArray
minecraft_msgs/msg/PlayerStatus
minecraft_msgs/msg/Item
minecraft_msgs/msg/Block
minecraft_msgs/msg/MobCategory
```

### show
By running `ros2 interface show message_type`, you can inspect the contents of that type.

```bash
    $ ros2 interface show minecraft_msgs/msg/Item 
string name
uint8 count
int16 damage
int16 max_damage
```
