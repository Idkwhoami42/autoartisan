# Mason Camera

## Description

This folder contains the code for the camera node that processes the images from the camera and publishes them to `/detction` topic.

## Usage

To run the camera node, you must first have the realsense package running. Then, you can run the following command
```bash
ros2 run mason_camera camera_node
```

The node will publish the images to `/detection` topic.