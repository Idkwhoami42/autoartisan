# Mason Hardware Package

This package includes the implementation of the hardware interface for ros2_control. It also includes the code for reading the contact sensors and publishing the state of the motors. Most of the business logic is similar to the `vesc-cpp` folder which is used for the vesc firmware.

## Usage

This package is used by the `mason_control` package. You can use the launch file in the `mason_control` package to launch the hardware interface along with the robot controller.

## File Structure

- `src/mason_interface.cpp`: This file contains the implementation of the hardware interface for ros2_control.
- `src/motors.cpp`: Logic for controlling the motors (Similar to `vesc-cpp/src/motors.cpp`).
- `src/sensors.cpp`: Logic for reading the contact sensors via GPIO.