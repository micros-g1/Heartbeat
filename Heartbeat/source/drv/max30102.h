/*
 * max30102.h
 *
 *  Created on: 14 Nov 2020
 *      Authors: grein, taomasgonzalez
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
max30102_state_t max30102_init(max30102_mode_t initial_mode);
max30102_state_t max30102_get_revision_id(uint8_t* rev_id);
max30102_state_t max30102_get_part_id(uint8_t* part_id);

//temperature readings
max30102_state_t max30102_trigger_temp_read();
max30102_state_t max30102_is_temp_read_ready(bool* rdy);
max30102_state_t max30102_wait_temp_read_ready();
max30102_state_t max30102_get_temperature_c(float* temp);

//spO2 readings
max30102_state_t max30102_set_fifo_config(max30102_fifo_configuration_t *config);
max30102_state_t max30102_set_spo2_config(max30102_spo2_configuration_t *config);
max30102_state_t max30102_set_led_current(uint8_t curr_lvl,  max30102_addr_t led_addr);
#endif /* DRV_MAX30102_H_ */
