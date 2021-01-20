/*
 * ad8232.h
 *
 *  Created on: 13 ene. 2021
 *      Author: Tomas
 */

#ifndef DRV_AD8232_H_
#define DRV_AD8232_H_


#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	AD8232_SUCCESS = true,
	AD8232_FAILURE = false
}ad8232_state_t;

ad8232_state_t ad8232_init();
ad8232_state_t ad8232_set_sampling_period(uint64_t micro_segs);
ad8232_state_t ad8232_trigger_reads();
ad8232_state_t ad8232_stop_reading();
uint32_t ad8232_get_new_sample();

#endif /* DRV_AD8232_H_ */
