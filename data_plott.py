#!/usr/bin/env python3

import serial  # pip install pyserial
#import serial.tools.list_ports  # list ports
import matplotlib.pyplot as plt

# Configure the serial port settings
ser = serial.Serial('/dev/cu.usbmodem14103', baudrate=115200, timeout=0.1)  # Replace with the correct port
print("connected")
# Initialize lists to store data
data = []

# Read data from the serial port
try:
    while True:
        #ser = serial.Serial()
        line = ser.readline().decode().strip()  # Read a line of text from the serial port and trim whitespace
        if line != "":
            number = float(line)  # Convert the received string to an integer
            data.append(number)  # Append the received number to the data list

            # Plot the data in real-time
            plt.plot(data)
            plt.xlabel('Time')
            plt.ylabel('Value')
            plt.title('Received Data Plot')
            plt.pause(0.1)  # Pause for a short time to update the plot
            plt.clf()  # Clear the plot for the next iteration

except KeyboardInterrupt:
    # Exit the loop gracefully on Ctrl+C
    pass

# Close the serial port when done
ser.close()
