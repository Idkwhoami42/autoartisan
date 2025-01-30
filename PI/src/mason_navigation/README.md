# Mason Navigation

## Description

This folder contains the code for the path finding and navigation of the mason robot. There is single ROS2 node that subscribes to `/detection` topic from the mason_camera package and computes the path for the robot to follow.

## Usage

To run the navigation node, you must first have the mason_camera package running. Then, you can run the following command
```bash
ros2 run mason_navigation navigation_node
```

The node will publish the path to `/path` topic and the current position to `/forward_position_controller/commands` topic.