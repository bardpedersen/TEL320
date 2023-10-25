/*
 * estimated_water_speed.c
 *
 *  Created on: Oct 25, 2023
 *      Author: Bård Pedersen
 */

#include <stdio.h>
#include <math.h>
#include "estimated_water_speed.h"

double water_flow(double higth, double radius, double K, double S, double n, int number_peaks) {
    double theta, R, A, Q;

    if (higth == 2 * radius || number_peaks == 1) { // Full pipe
        R = radius / 2; // Hydraulic radius
        A = M_PI * pow(radius, 2); // Area of the pipe
    } 
    else {
        theta = 2 * acos((radius - higth) / radius);
        R = (pow(radius, 2) * ((theta - sin(theta)) / 2)) / (radius * theta); // Hydraulic radius
        A = pow(radius, 2) * ((theta - sin(theta)) / 2);
    }

    Q = (K * A * pow(R, 2.0/3.0) * sqrt(S)) / n; // Flow rate in m^3/s
    return Q;
}
