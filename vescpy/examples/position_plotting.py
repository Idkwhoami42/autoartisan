import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from motor import Motor
import time
import threading
import matplotlib.pyplot as plt

def plot_position(motors):
    plt.ion()
    fig, ax = plt.subplots()
    start_time = time.time()
    times = []
    positions_1 = []
    positions_2 = []
    currents = []

    while not stop_plotting:
        if not motors[0].shutdown:
            times.append(time.time() - start_time)
            positions_1.append(motors[0].pos)
            positions_2.append(motors[1].pos)
            currents.append(motors[0].avg_in_current)

        ax.clear()

        ax.plot(times, positions_1, label='Position', color='orange')
        ax.plot(times, positions_2, label='Position', color='blue')
        ax.plot(times, currents, label="Current", color='red')
        # ax.plot(times, [positions_2[i]-positions_1[i] for i in range(len(positions_1))], label='Error', color='green')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (deg)')
        ax.set_title('Wheel position vs time (offset = {}, rotations = {})'.format(motors[0].offset, motors[0].rotations))
        ax.grid(True)

        ax.legend()

        plt.pause(0.02)

serialport = "/dev/ttyACM0"

main_motor = Motor(72, direction=0, is_can=False, can_ids=[125], serialport=serialport)
other_motor = Motor(125, direction=1, is_can=True, serialport=serialport)
stop_plotting = False

def rotate_test(motors):
    global stop_plotting
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        for motor in motors:
            motor.do_homing(ser)
            motor.get_values(ser)

        time.sleep(1)
        motors[0].go_to_pos(ser, 200, degrees_per_step=0.1, velocity=100, velocity_rampsteps=None, can_motors=motors[1:])
        time_now = time.time()
        while time.time() - time_now < 0.1:
            motors[0].get_values(ser)
            motors[1].get_values(ser)

        motors[0].go_to_pos(ser, 0, degrees_per_step=0.1, velocity=50, velocity_rampsteps=100, can_motors=motors[1:])
        motors[0].shutdown = True

pos_thread = threading.Thread(target=rotate_test, args=([main_motor, other_motor],))
pos_thread.start()

plot_position([main_motor, other_motor])

# rotate_test([main_motor, other_motor])
