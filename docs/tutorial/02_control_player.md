# Control Player

Let's control the player remotely using Python.

On this page, you will learn the following:

* Creating a package and a node
* Writing a Python publisher

## 1. Create a Package

Move to a suitable workspace:

```bash
mkdir -p ~/minecraft_ros2_tutorial_ws/src
cd ~/minecraft_ros2_tutorial_ws/src
```

Run the following command to create a package template with a Python node:

```bash
ros2 pkg create --build-type ament_python minecraft_ros2_tutorial
```

Modify the template as needed.

### package.xml

Add package information to **package.xml**.

Since this tutorial depends on `geometry_msgs`, add it as a dependency.

Insert the following line between `<test_depend>` and `<export>`:

```xml
<depend>geometry_msgs</depend>
```

Also, update the `description`, `maintainer`, and `license` tags.

### setup.py

The **setup.py** file contains Python-related configuration.

Register the node by editing the `console_scripts` section near the end:

```py
entry_points={
        'console_scripts': [
                'control_player = minecraft_ros2_tutorial.control_player:main',
        ],
},
```

## 2. Write the Python Code

Create `minecraft_ros2_tutorial/control_player.py` and add the following code:

```py
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ControlPlayer(Node):
    def __init__(self):
        super().__init__('control_player') # Create a Node named control_player
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) # Publisher for /cmd_vel
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback) # Call timer_callback every 0.01s
        self.speed = 1.0
        print("Control Player Node Initialized. Use wasd keys to move, Space to jump, WASD for rotation.")

    def timer_callback(self):
        key = input("enter key (wasd,space,WASD) >>>")
        twist = Twist() # Create message

        # Assign values based on input
        if 'w' in key:
            twist.linear.x += self.speed
        if 's' in key:
            twist.linear.x -= self.speed
        if 'a' in key:
            twist.linear.y += self.speed
        if 'd' in key:
            twist.linear.y -= self.speed
        if ' ' in key:
            twist.linear.z = 1.0
        if 'W' in key:
            twist.angular.y += self.speed
        if 'S' in key:
            twist.angular.y -= self.speed
        if 'A' in key:
            twist.angular.z += self.speed
        if 'D' in key:
            twist.angular.z -= self.speed

        self.publisher_.publish(twist) # Publish message


def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2

    control_player = ControlPlayer()

    rclpy.spin(control_player) # Start infinite loop

    # Cleanup on shutdown
    control_player.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

This code allows you to send movement commands: `wasd` for translation, **Space** for jumping, and **WASD** for rotation.

Now, let’s try running it! (Make sure **minecraft\_ros2** is launched and you have joined a world.)

Move to the workspace and build:

```bash
cd ~/ros2_tutorial_ws
colcon build
```

For the first run, source the setup file:

```bash
source install/setup.bash
```

Then, run the node:

```bash
ros2 run minecraft_ros2_tutorial control_player
```

Now, you can remotely control the Minecraft player through ROS 2!
