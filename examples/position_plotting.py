import serial
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from motor import Motor
import time
import threading
import matplotlib.pyplot as plt

def plot_position(motor):
    plt.ion()
    fig, ax = plt.subplots()
    start_time = time.time()
    times = []
    positions = []

    while not stop_plotting:
        if not motor.shutdown:
            times.append(time.time() - start_time)
            positions.append(motor.pos)

        ax.clear()

        ax.plot(times, positions, label='Position', color='orange')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (deg)')
        ax.set_title('Wheel position vs time (offset = {}, rotations = {})'.format(motor.offset, motor.rotations))
        ax.grid(True)
        ax.legend()

        plt.pause(0.02)

serialport = "/dev/ttyACM0"

main_motor = Motor(1, is_can=False, serialport=serialport)
stop_plotting = False

def rotate_test(motor):
    global stop_plotting
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        motor.do_homing(ser)
        print(motor.offset)
        motor.go_to_pos(ser, 800, degrees_per_step=0.1, velocity=100, velocity_rampsteps=100)
        time_now = time.time()
        while time.time() - time_now < 1:
            motor.get_values(ser)

        motor.go_to_pos(ser, 0, degrees_per_step=0.1, velocity=50, velocity_rampsteps=50)
        motor.shutdown = True

pos_thread = threading.Thread(target=rotate_test, args=(main_motor,))
pos_thread.start()

plot_position(main_motor)
