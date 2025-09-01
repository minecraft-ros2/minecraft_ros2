# Topic

In ROS 2, topics are an essential communication method.

On this page, you will learn the following:

* message
* publisher, subscriber
* `ros2 topic` command
* `ros2 interface` command

## Message

The information exchanged over topics is called a **message**.

You can freely define the type of a message.

For example, using `msg/Bool` from the `std_msgs` package allows you to communicate boolean values over a topic.

In **minecraft\_ros2**, custom message types are defined in the `minecraft_msgs` package.

## Publisher and Subscriber

The sender is called a **publisher**, and the receiver is called a **subscriber**.

In other words, when a publisher sends a message to a topic, any subscriber that listens to the same topic will receive it and perform some action.

## `ros2 topic` Command

The `ros2 topic` command allows you to inspect and interact with topics.

Here, we introduce the frequently used subcommands: `list`, `echo`, `info`, and `pub`.

### list

Running `ros2 topic list` shows a list of currently available topics.

With the `-v` option, you can see more detailed information.

For example, after launching **minecraft\_ros2** and joining the world, you should see output like this:

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

Running `ros2 topic echo <topic_name>` lets you monitor the data flowing through a topic.

For instance, after launching **minecraft\_ros2**, try:

```bash
    $ ros2 topic echo /player/status
name: Dev
dimension: minecraft:overworld
food_level: 20
score: 0
.....
```

This shows the player's status.

### info

Running `ros2 topic info <topic_name>` provides information about the publishers and subscribers of that topic.

```bash
    $ ros2 topic info /player/status
Type: minecraft_msgs/msg/PlayerStatus
Publisher count: 1
Subscription count: 0
```

### pub

Running `ros2 topic pub <topic_name> <type> <message>` publishes a message to a topic.

For example, to send a velocity command of 1 m/s forward and 0.3 m/s to the left on `/cmd_vel`:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.3}}"
```

The message type `geometry_msgs/msg/Twist` represents translation and rotation in 3D. Here, `linear.x` corresponds to forward translation, and `linear.y` to leftward translation.

Since this command continues publishing, you can stop it with **Ctrl + C**. Then, publish an empty message to stop the movement.

If you omit the message argument, an empty message will be sent.

## `ros2 interface` Command

The `ros2 interface` command provides information about message definitions. Commonly used subcommands are `package` and `show`.

### package

Running `ros2 interface package <package_name>` lists the messages available in that package.

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

Running `ros2 interface show <message_type>` displays the structure of that message.

```bash
    $ ros2 interface show minecraft_msgs/msg/Item
string name
uint8 count
int16 damage
int16 max_damage
```
