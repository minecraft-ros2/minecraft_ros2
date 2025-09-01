# Break Block

Let’s use Python to control the player and break surrounding blocks.

On this page, you will learn the following:

* Quaternions
* Service communication
* Position P-control

## 1. Try Service Communication

Service communication is a synchronous/asynchronous communication method where a server and client exchange requests and responses.

In **minecraft\_ros2**, services are used for breaking blocks, executing in-game commands, summoning entities, and more.

First, run the client and join a world.

Check the list of available services:

```bash
ros2 service list
```

In this tutorial, we will use the `/dig_block` service.

Aim at a block in the player’s view and run:

```bash
ros2 service call /dig_block minecraft_msgs/srv/DigBlock
```

If the block breaks, it worked!

## 2. Get the Player’s Orientation

Now you can break blocks, but you also need to aim at them. Let’s automate that.

For this tutorial, we will use debug messages to get the player’s pose.

The `/player/ground_truth` topic publishes the player’s position and orientation. Try:

```bash
ros2 topic echo /player/ground_truth
```

Example output:

```bash
---
position:
  x: 14.988304348096932
  y: -589.1905229054375
  z: 74.0
orientation:
  x: 0.025652511045336723
  y: 0.03142489492893219
  z: -0.6318491101264954
  w: 0.7740291357040405
---
```

This gives you the position (x, y, z) and orientation (x, y, z, w).

That extra **w**? This is called a **quaternion**, which represents rotation without gimbal lock.

You can also convert a quaternion into roll-pitch-yaw (Euler angles).

## 2. Add New Code

Create a new file at:

`~/minecraft_ros2_tutorial_ws/src/minecraft_ros2_tutorial/minecraft_ros2_tutorial/break_block.py`

and add the following code:

```py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import math

class BreakBlock(Node):
    def __init__(self):
        super().__init__('break_block')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, '/player/ground_truth', self.pose_callback, 10
        ) # Call pose_callback when receiving /player/ground_truth

        self.player_pose = None
        self.Kp = 2.0
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Break Block Node Initialized.")

    def pose_callback(self, msg: Pose):
        self.player_pose = msg

    def quaternion_to_yaw_pitch(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = -math.atan2(siny_cosp, cosy_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        return yaw, pitch

    def timer_callback(self):
        if self.player_pose is None:
            return

        yaw, pitch = self.quaternion_to_yaw_pitch(self.player_pose.orientation)

        self.target_block_position = [1, 0, -1] # x: forward, y: left, z: down
        dx, dy, dz = self.target_block_position
        dz = dz - 1 # Adjust for player’s head offset
        target_yaw = math.atan2(-dy, dx)
        target_pitch = math.atan2(-dz, math.sqrt(dx**2 + (-dy)**2))

        yaw_error = target_yaw - yaw
        pitch_error = target_pitch - pitch

        twist = Twist()
        twist.angular.y = -self.Kp * pitch_error
        twist.angular.z = -self.Kp * yaw_error
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BreakBlock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Add this line to the `console_scripts` section in **setup.py**:

```py
'break_block = minecraft_ros2_tutorial.break_block:main',
```

Then, rebuild and source:

```bash
colcon build
source install/setup.bash
```

Run the node:

```bash
ros2 run minecraft_ros2_tutorial break_block
```

Now the player’s view automatically points to `target_block_position`. This is achieved using coordinate conversion and P-control.

* `target_block_position` is the block’s relative position (here: forward + downward).
* P-control means the control command is proportional to the error.

Try changing `target_block_position` and observe the behavior.

## 3. Break a Specific Block

Next, let’s break the block automatically and move forward, creating a staircase.

Since breaking blocks pauses the game, a special flag is used here.

```py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from minecraft_msgs.srv import DigBlock
import math

class BreakBlock(Node):
    def __init__(self):
        super().__init__('break_block')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose, '/player/ground_truth', self.pose_callback, 10
        )

        # DigBlock service client
        self.dig_client = self.create_client(DigBlock, '/dig_block')
        while not self.dig_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dig_block service...')

        self.player_pose = None
        self.in_progress = False
        self.Kp = 2.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Break Block Node Initialized.")

    def pose_callback(self, msg: Pose):
        self.player_pose = msg

    def quaternion_to_yaw_pitch(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = -math.atan2(siny_cosp, cosy_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        return yaw, pitch

    def timer_callback(self):
        if self.player_pose is None:
            return

        yaw, pitch = self.quaternion_to_yaw_pitch(self.player_pose.orientation)

        self.target_block_position = [1, 0, -1]
        dx, dy, dz = self.target_block_position
        target_yaw = math.atan2(-dy, dx)
        target_pitch = math.atan2(-dz, math.sqrt(dx**2 + (-dy)**2))

        yaw_error = target_yaw - yaw
        pitch_error = target_pitch - pitch

        twist = Twist()
        twist.angular.y = -self.Kp * pitch_error
        twist.angular.z = -self.Kp * yaw_error
        self.publisher_.publish(twist)

        if abs(yaw_error) < 0.05 and abs(pitch_error) < 0.05:
            if not self.in_progress:
                self.in_progress = True
                self.get_logger().info('Target reached, sending dig command...')
                twist = Twist()
                self.publisher_.publish(twist)
                req = DigBlock.Request()
                future = self.dig_client.call_async(req)
                future.add_done_callback(self.dig_response_callback)

    def dig_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'DigBlock response received: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        self.in_progress = False

        twist = Twist()
        twist.linear.x = 1.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BreakBlock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Also, add the dependency in **package.xml**:

```xml
<depend>minecraft_msgs</depend>
```

When you run this, the player will automatically dig blocks and move forward, creating a staircase!
