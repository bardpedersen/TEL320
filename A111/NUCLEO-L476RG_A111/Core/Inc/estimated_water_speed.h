/*
 * estimated_water_speed.h
 *
 *  Created on: Oct 25, 2023
 *      Author: Bård Pedersen
 */
#include <stdio.h>

#ifndef ESTIMATED_WATER_SPEED_H
#define ESTIMATED_WATER_SPEED_H

uint16_t water_flow(double higth, double radius, double K, double S, double n, uint16_t number_peaks);

#endif
