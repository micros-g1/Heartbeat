/*
 * max30100.h
 *
 *  Created on: 29 Oct 2020
 *      Author: grein
 */

#ifndef DRV_MAX30100_H_
#define DRV_MAX30100_H_

#include <stdbool.h>
#include "max30100_registers.h"

typedef enum
{
	MAX30100_SUCCESS = 1,
	MAX30100_FAILURE = 0
}max30100_state_t;

//TODO: Documentation
max30100_state_t max30100_init();
max30100_state_t max30100_get_almost_full_flag(bool* flag);
max30100_state_t max30100_get_temp_ready_flag(bool* flag);
max30100_state_t max30100_get_hr_ready_flag(bool* flag);
max30100_state_t max30100_is_power_ready(bool* flag);
max30100_state_t max30100_fifo_almost_full_interrupt(bool enable);
max30100_state_t max30100_fifo_almost_full_get_interrupt_enabled(bool* is_enabled);
max30100_state_t max30100_temp_ready_interrupt(bool enable);
max30100_state_t max30100_temp_ready_get_interrupt_enabled(bool* is_enabled);
max30100_state_t max30100_hr_ready_interrupt(bool enable);
max30100_state_t max30100_hr_ready_get_interrupt_enabled(bool* is_enabled);
max30100_state_t max30100_set_fifo_write_pointer(uint8_t p_val);
max30100_state_t max30100_get_fifo_write_pointer(uint8_t * p_val);
max30100_state_t max30100_set_fifo_read_pointer(uint8_t p_val);
max30100_state_t max30100_get_fifo_read_pointer(uint8_t * p_val);
max30100_state_t max30100_get_fifo_overflow_counter(uint8_t* counter);
max30100_state_t max30100_fifo_read_next();
max30100_state_t max30100_get_current_fifo_sample_ir(uint16_t* sample);
max30100_state_t max30100_get_current_fifo_sample_red(uint16_t* sample);
max30100_state_t max30100_clear_fifo();
max30100_state_t max30100_get_fifo_samples_count(uint8_t* samples_count);
max30100_state_t max30100_shutdown_mode(bool shdn);
max30100_state_t max30100_get_shutdown_mode_enabled(bool* shdn);
max30100_state_t max30100_reset();
max30100_state_t max30100_get_reset_ready(bool* ready);
max30100_state_t max30100_wait_reset_ready();
max30100_state_t max30100_trigger_temp_read();
max30100_state_t max30100_get_temperature_ready(bool* ready);
max30100_state_t max30100_wait_temperature_ready();
max30100_state_t max30100_set_operating_mode(max30100_mode_t mode);
max30100_state_t max30100_get_operating_mode(max30100_mode_t* mode);
max30100_state_t max30100_spo2_high_resolution(bool hi_res);
max30100_state_t max30100_spo2_get_high_resolution_enabled(bool* hi_res);
max30100_state_t max30100_spo2_set_sample_rate(max30100_spo2_sr_t sr);
max30100_state_t max30100_spo2_get_sample_rate(max30100_spo2_sr_t* sr);
max30100_state_t max30100_set_led_pulse_width(max30100_led_pw_t pw);
max30100_state_t max30100_get_led_pulse_width(max30100_led_pw_t* pw);
max30100_state_t max30100_set_red_led_current(max30100_led_cc_t cc);
max30100_state_t max30100_get_red_led_current(max30100_led_cc_t* cc);
max30100_state_t max30100_set_ir_led_current(max30100_led_cc_t cc);
max30100_state_t max30100_get_ir_led_current(max30100_led_cc_t* cc);
max30100_state_t max30100_get_temperature_c(float* temp);
max30100_state_t max30100_get_revision_id(uint8_t* rev_id);
max30100_state_t max30100_get_part_id(uint8_t* part_id);



#endif /* DRV_MAX30100_H_ */
