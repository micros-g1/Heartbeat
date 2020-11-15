/*
 * max30102.h
 *
 *  Created on: 14 Nov 2020
 *      Author: grein
 */

#ifndef DRV_MAX30102_H_
#define DRV_MAX30102_H_
#include <stdbool.h>
#include "max30102_registers.h"

typedef enum
{
	MAX30102_SUCCESS = true,
	MAX30102_FAILURE = false
}max30102_state_t;

//TODO: COMMENT
max30102_state_t max30102_init();
max30102_state_t max30102_reset();
max30102_state_t max30102_is_reset_ready(bool* reset_ready);
max30102_state_t max30102_wait_reset_ready();
max30102_state_t max30102_get_revision_id(uint8_t* rev_id);
max30102_state_t max30102_get_part_id(uint8_t* part_id);


//temperature readings
max30102_state_t max30102_trigger_temp_read();
max30102_state_t max30102_is_temp_read_ready(bool* rdy);
max30102_state_t max30102_wait_temp_read_ready();
max30102_state_t max30102_get_temperature_c(float* temp);

#endif /* DRV_MAX30102_H_ */
