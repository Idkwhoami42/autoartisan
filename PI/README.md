# Mason Robot

## Description

This repository contains the code for the raspberry pi for the mason robot. The code is written for ROS2 Jazzy in both Python and C++. There is also a folder (src/mason_web) for the UI interface which uses TypeScript, React and Bun.
Code is divided into different packages for each of the components of the robot. More details about the packages can be found in the README.md file in each package.

## External Packages
This project depends on the following external ROS packages:
- [Realsense-Ros](https://github.com/intel/ros_realsense) For the realsense depth camera
- [Micro-ROS](https://github.com/micro-ROS) For the communication between the raspberry pi and the microcontroller

## Installation
You must have ROS2 Jazzy installed on your system and have updated the PATH variable. The project also uses `bun` for the web client.

## Usage
You can use the launch files in the mason package to launch the robot. It is also possible to run individual pacakges.

```bash
# building all the packages
colcon build
# source the packages
source install/setup.bash
# launch the robot
ros2 launch mason mason.launch.py
```

To run the web interface, you must first run the flask server and then run the client.
```bash
# starting the backend server
ros2 run mason_flask flask_server

# starting the client
cd src/mason_web
bun i
bun run dev --host autoartisan.local
```

The website will be available at http://autoartisan.local:5173

## Style Guide and Linting
We use clang-format to format the code. Each package has a .clang-format file that can be used to format the code in that package. The style guide is also described in the README.md file in each package.