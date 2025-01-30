# Teensy 

## Description

This folder contains the code for the Teensy 4.1 board. The code is written in C++ and uses the Arduino framework. It uses micro-ROS for communication with the Raspberry PI. The microcontroller is responsible for controlling the servos and extruding motor. For communication, it subscribes to the `/mason_fsm_publisher/state` topic where the state of the robot is published. 

### States
- 0: Start brush
- 1: Stop brush
- 2: Start vertical smoothing
- 3: Stop vertical smoothing
- 4: Start horizontal smoothing
- 5: Stop horizontal smoothing
- 6: Extrude
- 7: Retract


## Usage

To run the code, you must first have the Raspberry PI running with the micro-ROS agent. Then using PlatformIO, you can run the following command.
```bash
pio build
pio run
```

