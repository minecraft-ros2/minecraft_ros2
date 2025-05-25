#!/bin/bash
# 適宜URIとポジションを変更すること！
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity '{"name":"diffbot","allow_renaming":false,"uri":"/ws/minecraft_ros2_dev/samples/diffbot.geo.json","resource_string":"","entity_namespace":"","initial_pose":{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":""},"pose":{"position":{"x":-14.0,"y":76.0,"z":14.0},"orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}}}}'

