# AutoArtisan

## Description

This repository contains the source code for the Mason robot. There are 2 main components:
- ROS packages: These are run on the raspberry pi and provide the robot's functionality.
- Arduino code: This is ran on the Teensy 4.1 and provides the motor control of the tool head and sensor readings.

## Folder Structure

- PI/: Contains the ROS packages for the robot.
- Teensy/: Contains the Arduino code (PlatformIO) for the robot.
- Vesc-cpp/: Contains the code for interfacing with the Vesc firmware.

You can find more information about the code in the README.md files in each folder.
