#!/usr/bin/env python3 

import numpy as np

def water_flow(higth, radius, K, S, n, number_peaks):
    
    if higth == 2 * radius or number_peaks == 1: # Full pipe
        R = radius / 2 # Hydraulic radius 
        A = np.pi * radius**2 # Area of the pipe 
    else:
        theta = 2 * np.arccos((radius - higth) / radius)
        R = (radius**2 * ((theta - np.sin(theta))/2)) / (radius * theta) # Hydraulic radius
        A = radius**2 * ((theta - np.sin(theta))/2)

    print("area", A)
    print("Hyd raid", R)
    Q = (K*A*R**(2/3) *S**(1/2)) / n # Flow rate in m^3/s
    return Q 

def convert(number):
    a = 20
    b = 50
    c = a+ b
    d = (c-a)/(1000)
    return number * d

if __name__ == "__main__":
    higth = 1 # m Given by 
    radius = 25 # m

    K = 1.49 # Constan, 
    S = 0.001 # Slope of the pipe, assumption.
    n = 0.20 # Roughness coefficient, assumption.
    Q = water_flow(higth, radius, K, S, n, 2)
    print(Q)

    # vanntank er 23 cm
    # vann er 28 cm
    # Bord er 35 cm
    print(convert(1))
