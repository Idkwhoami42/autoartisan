import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from motor import Motor

serialport = "/dev/ttyACM0"

motor_id = 0 # The VESC ID that you can specify under "App Settings -> VESC ID" 
motor = Motor(motor_id, serialport=serialport)

with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
  motor.do_homing(ser) # Homing is used to always start at 0, regardless of the actual starting position.
  motor.go_to_pos(ser, 360) # Position is measured in degrees, so this command will spin the motor one full rotation.
  motor.shutdown()
