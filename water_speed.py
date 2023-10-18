#!/usr/bin/env python3 

import math

def water_velocity(height):
    # Constants
    g = 9.81  # Acceleration due to gravity in m/s²

    # Calculate the velocity using Torricelli's law
    velocity = math.sqrt(2 * g * height)
    
    return velocity

# Example usage:
water_height = 0.2  # Replace with the actual height in meters
velocity = water_velocity(water_height)
print(f"Water velocity for a height of {water_height} meters is approximately {velocity:.2f} m/s.")

