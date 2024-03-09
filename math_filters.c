/*
 * math_filters.c
 *
 *  Created on: Mar 7, 2024
 *      Author: maxgu
 */

#include "math_filters.h"

uint16_t adc_value_samples[12];
int sample_count = 0;

double filtered_average(uint16_t *samples)
{
    uint16_t min = 5000;
    uint16_t max = 0;

    int idx1, idx2;

    for (int i = 0; i < 12; ++i)
    {
        if (samples[i] > max)
        {
            max = samples[i];
            idx1 = i;
        }
        else if(samples[i] < min)
        {
            min = samples[i];
            idx2 = i;
        }
    }

    uint16_t sum = 0;
    double avg;
    // compute average
    for (int i = 0; i < 12; ++i)
    {
        if (i != idx1 && i != idx2)
        {
            sum += samples[i];
        }
    }
    avg = sum/10;

    return avg;
}

double adc_filtered_average(uint16_t *samples, uint16_t adc_value)
{
	adc_value_samples[sample_count % 12] = adc_value;

    if (sample_count >= 12)
    {
    	++sample_count;
    	return filtered_average(adc_value_samples);
    } // exclude erroneous large values sporadically seem (Temporary workaround)
    else if(sample_count > 214700000)
    {
        sample_count = (sample_count % 12);
        return filtered_average(adc_value_samples);
    }
    else
    {
    	++sample_count;
        return adc_value;
    }
}
