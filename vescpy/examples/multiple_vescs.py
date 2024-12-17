import serial
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
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