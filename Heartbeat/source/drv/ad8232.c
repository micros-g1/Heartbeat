/*
 * ad8232.c
 *
 *  Created on: 13 ene. 2021
 *      Author: Tomas
 */

#include "ad8232.h"
#include <stddef.h>

static unsigned int curr_buffer_len = 0;
static float buffer[AD8232_MAX_SAMPLES];

ad8232_state_t ad8232_init(){
	//init ADC, init PIT but don't start triggering events
	return AD8232_SUCCESS;
}

ad8232_state_t ad8232_trigger_reads(){
	//start PIT to trigger the ADC and empty the samples' buffer
	curr_buffer_len = 0;
	return AD8232_SUCCESS;
}
ad8232_state_t ad8232_stop_reading(){
	//stop PIT
	return AD8232_SUCCESS;
}

uint8_t ad8232_get_num_available_samples(){
	return curr_buffer_len;
}

uint8_t ad8232_get_n_samples(uint8_t n_samples, float *samples){

	//if there are not enough samples in the buffer, just read all provide all available samples
	if(curr_buffer_len < n_samples) n_samples = curr_buffer_len;

	int i, j=0;
	//copy n_samples from ad8232 buffer to the user provided buffer.
	for(i = 0; i < n_samples; i++)
		samples[i] = buffer[i];

	//put whatever samples are left back in the ad8232 buffer to the beginning of the buffer.
	while(i < curr_buffer_len)
		buffer[j++] = buffer[i++];

	//update buffer size with whatever is left on the ad8232 buffer.
	curr_buffer_len = (n_samples < curr_buffer_len) ? curr_buffer_len - n_samples : 0;

	return n_samples;
}
