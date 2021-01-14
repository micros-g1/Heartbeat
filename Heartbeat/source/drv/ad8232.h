/*
 * ad8232.h
 *
 *  Created on: 13 ene. 2021
 *      Author: Tomas
 */

#ifndef DRV_AD8232_H_
#define DRV_AD8232_H_

//should be an even number!
#define AD8232_MAX_SAMPLES 50
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	AD8232_SUCCESS = true,
	AD8232_FAILURE = false
}ad8232_state_t;

ad8232_state_t ad8232_init();
ad8232_state_t ad8232_trigger_reads();
ad8232_state_t ad8232_stop_reading();
ad8232_state_t ad8232_get_new_sample(float *sample);

#endif /* DRV_AD8232_H_ */
