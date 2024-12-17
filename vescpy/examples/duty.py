import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from motor import Motor
import time

serialport = "/dev/ttyACM0"

motor_id = 0 # The VESC ID that you can specify under "App Settings -> VESC ID" 
motor = Motor(motor_id, serialport=serialport)

with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
  motor.set_duty(ser, 0.2)
  time.sleep(1)
  motor.shutdown() # Not needed but good practice
