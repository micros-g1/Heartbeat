/*
 * max30205.h
 *
 *  Created on: 13 ene. 2021
 *      Author: Tomas
 */

#ifndef DRV_MAX30205_H_
#define DRV_MAX30205_H_
#include <stdbool.h>
#include "max30205_registers.h"

typedef enum
{
	MAX30205_SUCCESS = true,
	MAX30205_FAILURE = false
}max30205_state_t;

//Power-On Reset Value
/*
 * These default POR values correspond to the following modes of operation:
 	 * Comparator mode
 	 * OS active low
 	 * 1 fault, fault queue
	 * Normal data format
	 * Timeout enabled for MAX30205
 */

max30205_state_t max30205_init();

//max30205 configuration
max30205_state_t max30205_set_config(max30205_config_t *config);
max30205_state_t max30205_set_one_shot(bool one_shot);
max30205_state_t max30205_set_not_timeout(bool not_timeout);
max30205_state_t max30205_set_data_format(bool data_format);
max30205_state_t max30205_set_fault_queue(max30205_fault_queue_t fault_queue);
max30205_state_t max30205_set_os_polarity(bool smp_ave);
max30205_state_t max30205_not_comparator_interrupt(bool smp_ave);
max30205_state_t max30205_shutdown(bool smp_ave);

max30205_state_t max30205_temp_read(float *temp);



#endif /* DRV_MAX30205_H_ */
