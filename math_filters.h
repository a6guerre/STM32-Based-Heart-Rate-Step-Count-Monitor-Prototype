/*
 * mathfilters.h
 *
 *  Created on: Mar 7, 2024
 *      Author: maxgu
 */

#ifndef SRC_MATH_FILTERS_H_
#define SRC_MATH_FILTERS_H_
#include <stdint.h>
#include <math.h>

// removed the lowest and highest value from sample set, and then computes avg. of resultant filtered set
double filtered_average(uint16_t *samples);
// Moving filter for ADC
double adc_filtered_average(uint16_t *samples, uint16_t adc_value);
#endif /* SRC_MATH_FILTERS_H_ */
