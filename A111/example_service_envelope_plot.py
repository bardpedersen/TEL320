#!/usr/bin/env python3 

import serial
import matplotlib.pyplot as plt
import re
import numpy as np

# Configure the serial port settings
ser = serial.Serial('/dev/cu.usbmodem14103', baudrate=115200, timeout=0.1)  # '/dev/cu.usbmodem14103' #'COM3'
print("connected")

# Define the pattern to match lines with timestamps
pattern = r'^\d{2}:\d{2}:\d{2}\.\d{3}'

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], []) 
Amplitude = 0
Depth = 0
peaks_x = 0
peaks_y = 0

while True:
    try:
        data = ser.readline().decode().strip()
        if data != '' and data != ' ':
            # Remove lines that match the timestamp pattern
            if not re.match(pattern, data) and re.match("Peaks", data):
                peaks_x = [float(x) for x in data.split()[1:]]
                peaks_y = [Amplitude[int(peaks_x[0])], Amplitude[int(peaks_x[1])]]

            elif not re.match(pattern, data):
                #Generate a sample radar signal (you should replace this with your real data)
                Amplitude = [float(x) for x in data.split()]
                Depth = [float(x) for x in range(len(Amplitude))]

            ax.clear()  # Clear the previous points
            ax.plot(Depth, Amplitude)  # Plot the new data
            ax.plot(peaks_x, peaks_y, 'ro')  # Plot the peak points
            ax.relim()  # Recalculate the data limits
            plt.pause(0.1)

    except KeyboardInterrupt:
        # Exit the loop gracefully on Ctrl+C
        # Close the serial port when done
        ser.close()
        plt.close()

# Show the plots
plt.ioff()  # Turn off interactive mode to prevent the plot window from closing immediately
plt.show()