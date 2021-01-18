/*
 * ad8232.c
 *
 *  Created on: 13 ene. 2021
 *      Author: Tomas
 */

#include "ad8232.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "board/peripherals.h"
#include "drivers/fsl_pit.h"
#include "drivers/fsl_adc16.h"

#define VREF_BRD 3.300
#define SE_12BIT 4096.0

static unsigned int curr_buffer_len = 0;
static float buffer[AD8232_MAX_SAMPLES];

ad8232_state_t ad8232_init(){
	//Both PIT and ADC are already initialized (peripherals.c)
	return AD8232_SUCCESS;
}

ad8232_state_t ad8232_set_sampling_period(uint64_t micro_segs){

	PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0,
			(uint64_t)((micro_segs * CLOCK_GetFreq(BUS_CLK)) / 1000000U));
	return AD8232_SUCCESS;
}

ad8232_state_t ad8232_trigger_reads(){
	//start PIT to trigger the ADC and empty the samples' buffer
	PIT_StartTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
	curr_buffer_len = 0;
	return AD8232_SUCCESS;
}
ad8232_state_t ad8232_stop_reading(){

	PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
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

ad8232_state_t ad8232_get_new_sample(float *sample){
	//<SDK_ROOT>/boards/<BOARD>/driver_examples/adc_etc for API example
	uint32_t g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC0_PERIPHERAL,
			ADC0_CH0_CONTROL_GROUP);
	*sample = (float)(g_Adc16ConversionValue * (VREF_BRD / SE_12BIT));

	return AD8232_SUCCESS;
}
