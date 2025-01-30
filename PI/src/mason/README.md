# Mason

## Description

This folder contains the launch file for the mason robot. It launches the necessary nodes for the robot to function.

## Usage

You can lunch all the nodes by running the following command
```bash
ros2 launch mason mason.launch.py
```

This will launch the following nodes:
- micro_ros_agent: This is the micro-ROS agent that connects to the robot's serial port.
- mason_navigation: This is the node that computes the path for the robot to follow.
- mason_camera: This is the node that processes the images from the camera and publishes them to ROS.
- mason_flask: This is the node that provides the web interface for the robot.
- mason_control: This is the node that controls the robot's motors and reads the sensors.
