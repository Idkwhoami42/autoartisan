# Mason Flask Server

## Description

This folder contains the code for the flask server that communicates with the web interface and ROS2 nodes. It is written in Python and uses Flask.

## Usage

To run the flask server, you must first have the all the other packages/nodes running. Then, you can run the following command
```bash
ros2 run mason_flask flask_server
```

The server will start a flask server on port 5000 and will provide the web interface with the latest images from the camera and the detection images.