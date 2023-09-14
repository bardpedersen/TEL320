#!/usr/bin/env python3

import serial
import matplotlib.pyplot as plt

# Lists to store the temperature, pressure, and humidity data
temp_data = []
pres_data = []
hum_data = []

# Configure the serial port settings
ser = serial.Serial('/dev/cu.usbmodem14103', baudrate=115200, timeout=0.1)
print("connected")

# Create a figure with three subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

# Set titles and labels for the subplots
ax1.set_title('Temperature Data')
ax1.set_xlabel('Time')
ax1.set_ylabel('Temperature (°C)')
ax1.set_ylim(20, 30)

ax2.set_title('Pressure Data')
ax2.set_xlabel('Time')
ax2.set_ylabel('Pressure kPa')
ax2.set_ylim(100, 110)

ax3.set_title('Humidity Data')
ax3.set_xlabel('Time')
ax3.set_ylabel('Humidity (%)')
ax3.set_ylim(00, 50)

try:
    while True:
        data = ser.readline().decode().strip()
        if data !='':
            int_values = [float(x) for x in data.split(',')]
            # Append the received numbers to their respective data lists
            temp_data.append((int_values[0] / 100))
            pres_data.append((int_values[1] / (256.0 * 1000)))
            hum_data.append((int_values[2] / 1024))  # Assuming humidity is in percentage
            print(f'Temperature: {int_values[0] / 100} °C, Pressure: {int_values[1] / (256.0 * 1000)} kPa, Humidity: {(int_values[2] / 1024)} %')
            # Update the plots
            ax1.plot(temp_data, 'b-')
            ax2.plot(pres_data, 'r-')
            ax3.plot(hum_data, 'g-')

            plt.pause(0.1)  # Pause for a short time to update the plots

except KeyboardInterrupt:
    # Exit the loop gracefully on Ctrl+C
    pass

# Close the serial port when done
ser.close()

# Show the plots
plt.show()

