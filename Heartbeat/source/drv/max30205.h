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
max30205_state_t max30205_temp_read(float *temp);

#endif /* DRV_MAX30205_H_ */
