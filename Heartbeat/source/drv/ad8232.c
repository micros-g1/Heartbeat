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
	return AD8232_SUCCESS;
}
ad8232_state_t ad8232_stop_reading(){

	PIT_StopTimer(PIT_PERIPHERAL, PIT_CHANNEL_0);
	return AD8232_SUCCESS;
}

uint32_t ad8232_get_new_sample(){
	//<SDK_ROOT>/boards/<BOARD>/driver_examples/adc_etc for API example
	return ADC16_GetChannelConversionValue(ADC0_PERIPHERAL,
			ADC0_CH0_CONTROL_GROUP);
}
