# RViz2
Rviz is a 3D visualizer included with ROS (Robot Operating System).

By using Rviz, you can visualize data such as player information, LiDAR, and camera data in 3D space.

## Launch
```bash
rviz2
```
That's all it takes (assuming `source` has already been executed).
![Default screen](/images/tutorial/rviz2_default.png)

## Configuration
Here we will visualize Image and LiDAR data.
(Please ensure `minecraft_ros2` is running)

### Image
Image refers to images, and the Minecraft gameplay screen is being sent.

From the bottom left corner of the screen, click Add and select Image by topic.
![Selection screen](/images/tutorial/rviz2_add_image.png)

Then the Minecraft screen will be displayed in the bottom left corner!

![Displayed](/images/tutorial/rviz2_image.png)

### LiDAR
By [equipping a LiDAR sensor](/documentation/doc_sensors), you can determine the distance to surrounding blocks and entities.

Just like with Image, select Point Cloud 2 by topic.

You might expect something to appear, but nothing changes.

That's because the coordinate frame being displayed is incorrect.

By changing the Global Options' Fixed Frame to `player`, the data will be displayed.

![After change](/images/tutorial/rviz2_fixed_frame.png)

If it’s hard to see, you can adjust the Size (m) of Point Cloud 2 to make it easier to view.
