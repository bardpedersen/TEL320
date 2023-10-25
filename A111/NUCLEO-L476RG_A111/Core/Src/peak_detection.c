/*
 * peak_detection.c
 *
 *  Created on: Oct 11, 2023
 *      Author: pbuka
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>



struct Tuple{
	int index;
	uint16_t value;
};
int* mutliple_peak_detection(uint16_t *data, uint16_t data_length){

	uint16_t sum = 0;
	for (int i = 0; i < data_length; i++){
		sum = sum + data[i];

	}
	uint16_t baseline = sum/data_length;
	struct Tuple peaks[] = {
						   {0,0},
						   {0,0}
	};

	for (int i = 0;i<data_length-5;i++){
		if (data[i] > data[i-1] && data[i] > data[i+5]&& data[i] > baseline){
			if (data[i] > peaks[0].value){
				if (i - peaks[0].index > 10){
					peaks[1].index = peaks[0].index;
					peaks[1].value = peaks[0].value;
				}
				peaks[0].index = i;
				peaks[0].value = data[i];
			}

			else if (data[i] > peaks[1].value && i - peaks[0].index > 10){
				peaks[1].index = i;
				peaks[1].value = data[i];
			}
		}
	}
	int* peakIndexes = (int*)malloc(2 * sizeof(int));

	peakIndexes[0] = peaks[0].index;
	peakIndexes[1] = peaks[1].index;

	return peakIndexes;



}


