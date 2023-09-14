#!/usr/bin/env python3

import serial
import matplotlib.pyplot as plt
from PIL import Image
import serial.tools.list_ports
import numpy as np

# Lists to store the temperature, pressure, and humidity data
temp_data = []
pres_data = []
hum_data = []
"""
ports = list(serial.tools.list_ports.comports())

for p in ports:
    print(p)
"""
# Configure the serial port settings
ser = serial.Serial('COM3', baudrate=115200, timeout=0.1)
print("connected")
img = np.asarray(Image.open('pic.png'))
# Create a figure with three subplots
fig, axs = plt.subplots(2, 2)


axs[0, 0].set_title('Temperature Data')
axs[0, 0].set_xlabel('Time')
axs[0, 0].set_ylabel('Temperature (°C)')
axs[0, 0].set_ylim(20, 40)


axs[0, 1].set_title('Pressure Data')
axs[0, 1].set_xlabel('Time')
axs[0, 1].set_ylabel('Pressure (kPa)')
axs[0, 1].set_ylim(100, 110)


axs[1, 0].set_title('Axis [1, 0]')
axs[1, 0].set_xlabel('Time')
axs[1, 0].set_ylabel('Humidity (%)')
axs[1, 0].set_ylim(20, 70)
fig.tight_layout()

axs[1,1].imshow(img)
axs[1,1].axis('off')
# Set titles and labels for the subplots





try:
    while True:
        data = ser.readline().decode().strip()
        if data !='':
            int_values = [float(x) for x in data.split(',')]
            # Append the received numbers to their respective data lists
            print(data)
            temp_data.append((int_values[0] / 100))
            pres_data.append((int_values[1] / (256.0 * 1000)))
            hum_data.append((int_values[2] / 1024))  # Assuming humidity is in percentage
            print(f'Temperature: {int_values[0] / 100} °C, Pressure: {int_values[1] / (256.0 * 1000)} kPa, Humidity: {(int_values[2] / 1024)} %')
            # Update the plots
            axs[0,0].plot(temp_data, 'b-')
            axs[0,1].plot(pres_data, 'r-')
            axs[1,0].plot(hum_data, 'g-')

            plt.pause(0.1)  # Pause for a short time to update the plots

except KeyboardInterrupt:
    # Exit the loop gracefully on Ctrl+C
    pass

# Close the serial port when done
ser.close()

# Show the plots
plt.show()

