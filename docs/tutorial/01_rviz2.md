# RViz2

Rviz is a 3D visualizer that comes with ROS (Robot Operating System).

Using Rviz, you can display data such as player information, LiDAR, and camera streams in a 3D environment.

## Launch

```bash
rviz2
```

That’s all you need (assuming you have already sourced your environment).

![Default Screen](/images/tutorial/rviz2_default.png)

## Configuration

Here, we will visualize **Image** and **LiDAR** data. (Make sure **minecraft\_ros2** is running.)

### Image

Image topics carry image data—in this case, the Minecraft gameplay screen.

From the bottom-left **Add** button, choose **By topic → Image**.

![Selection Screen](/images/tutorial/rviz2_add_image.png)

Then, the Minecraft screen will appear in the bottom-left view!

![Displayed](/images/tutorial/rviz2_image.png)

### LiDAR

By [equipping a LiDAR sensor](/jp/documentation/doc_sensors), you can obtain distance measurements to surrounding blocks and entities.

Just like with Image, select **By topic → PointCloud2**.

At first, nothing may appear. This is because the coordinate frame is incorrect.

Change the **Global Options → Fixed Frame** to `player`, and the LiDAR data will be displayed.

![After Change](/images/tutorial/rviz2_fixed_frame.png)

If the points are hard to see, you can adjust **PointCloud2 → Size (m)** to make the visualization clearer.
