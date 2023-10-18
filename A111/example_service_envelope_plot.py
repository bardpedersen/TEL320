#!/usr/bin/env python3 

import serial
import matplotlib.pyplot as plt
import re
import numpy as np

# Configure the serial port settings
ser = serial.Serial('/dev/cu.usbmodem14203', baudrate=115200, timeout=0.1)  # '/dev/cu.usbmodem14103' #'COM3'
print("connected")

# Define the pattern to match lines with timestamps
pattern = r'^\d{2}:\d{2}:\d{2}\.\d{3}'

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], []) 


try:
    while True:
        data = ser.readline().decode().strip()
        if data != '' and data != ' ':
            # Remove lines that match the timestamp pattern
            if not re.match(pattern, data):
                #Generate a sample radar signal (you should replace this with your real data)
                Amplitude = [float(x) for x in data.split()]
                Depth = [float(x) for x in range(len(Amplitude))]
                line.set_data(Depth, Amplitude)  # Update the plot data
                ax.relim()  # Recalculate the data limits
                #print(test_speed.estimate_velocity(Amplitude, 512))
                plt.pause(0.1)
                

except KeyboardInterrupt:
    # Exit the loop gracefully on Ctrl+C
    # Close the serial port when done
    ser.close()
    plt.close()

# Show the plots
plt.ioff()  # Turn off interactive mode to prevent the plot window from closing immediately
plt.show()
