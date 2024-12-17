# VescPy
A Python implementation to control one or multiple VESCs over a single serial (USB) connection, tested with the latest firmware (v6.05). I made this for a custom robotics project and thus focused on position control, but it should be easy to add additional functions. I'm also not a Computer Scientist so some implementations may be poorly coded, but it works for our project.

## Example usage (Duty cycle)
```python
import serial
import time
from motor import Motor

serialport = "/dev/ttyACM0"

motor_id = 0 # The VESC ID that you can specify under "App Settings -> VESC ID" 
motor = Motor(motor_id, serialport=serialport)

with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
  motor.set_duty(ser, 0.2)
  time.sleep(1)
  motor.shutdown() # Not needed but good practice

```

## Example usage (Position control)
```python
import serial
from motor import Motor

serialport = "/dev/ttyACM0"

motor_id = 0 # The VESC ID that you can specify under "App Settings -> VESC ID" 
motor = Motor(motor_id, serialport=serialport)

with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
  motor.do_homing(ser) # Homing is used to always start at 0, regardless of the actual starting position.
  motor.go_to_pos(ser, 360) # Position is measured in degrees, so this command will spin the motor one full rotation.
  motor.shutdown()

```

## Example usage (Multiple VESCs over one serial connection)
You can control multiple VESCs over a single serial (USB) connection. To do this, connect the VESCs with each other using the CAN network and just create additional motor objects with the parameter `is_can` set to True. Finally, specify all the "CAN IDs" to the "main" motor.

```python
import serial
from motor import Motor

serialport = "/dev/ttyACM0"

# Here, the VESC with ID 2 is the "main" VESC, so this is the VESC that you connect to with an USB cable.
motors = [Motor(0, is_can=True), Motor(1, is_can=True), Motor(2, is_can=False, can_ids=[0, 1])]

with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
  motors[0].set_duty(ser, 0.2)
  motors[1].set_duty(ser, 0.1)
  motors[2].set_duty(ser, 0.3)
  time.sleep(1)
  motors[2].shutdown() # You only have to shutdown the main VESC, the other VESCs will be shutdown automatically
```
