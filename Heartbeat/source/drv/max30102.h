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

#define MAX30102_FIFO_MAX_SAMPLES 32

typedef enum
{
	MAX30102_SUCCESS = true,
	MAX30102_FAILURE = false
}max30102_state_t;


typedef union{
	uint8_t bytes[4];
	uint32_t led_data;
}__attribute__((__packed__, aligned(1))) max30102_led_data_t;

typedef struct{
	max30102_led_data_t led_data1;
	max30102_led_data_t led_data2;
}__attribute__((__packed__, aligned(1))) max30102_sample_t;

//checked> logic was checked, debug pending.
//debugged> code was tested on HW.

//TODO: COMMENT

//general functions
max30102_state_t max30102_init(max30102_mode_t initial_mode);
max30102_state_t max30102_get_revision_id(uint8_t* rev_id);
max30102_state_t max30102_get_part_id(uint8_t* part_id);

//interrupt config
max30102_state_t max30102_set_die_temp_rdy_en(bool die_tmp_rdy_en);						//checked
max30102_state_t max30102_set_alc_ovf_en(bool alc_ovf_en);								//checked
max30102_state_t max30102_set_ppg_rdy_en(bool ppg_rdy_en);								//checked
max30102_state_t max30102_set_a_full_en(bool a_full_en);								//checked

//interrupt status
max30102_state_t max30102_get_interrupt_status(max30102_interrupt_status_t *status);

//spo2 fifo config
max30102_state_t max30102_set_fifo_config(max30102_fifo_configuration_t *config);
max30102_state_t max30102_set_fifo_a_full(uint8_t fifo_a_full);							//checked
max30102_state_t max30102_set_fifo_roll_over_en(bool roll_over_en);						//checked
max30102_state_t max30102_set_fifo_smp_ave(max30102_smp_ave_t smp_ave);					//checked

//led config
max30102_state_t max30102_set_led_current(uint8_t curr_lvl,  max30102_addr_t led_addr);

//temperature readings
max30102_state_t max30102_trigger_temp_read();
max30102_state_t max30102_is_temp_read_ready(bool* rdy);
max30102_state_t max30102_wait_temp_read_ready();
max30102_state_t max30102_get_temperature_c(float* temp);

//spO2 config
max30102_state_t max30102_set_spo2_config(max30102_spo2_configuration_t *config);			//checked
max30102_state_t max30102_set_spo2_adc_rge(max30102_spo2_adc_resolution_t spo2_adc_rge);	//checked
max30102_state_t max30102_set_sr(max30102_spo2_sample_rate_t spo2_sample_rate);				//checked
max30102_state_t max30102_set_led_pw(max30102_led_pw_t led_pw);								//checked

//spO2 readings
max30102_state_t max30102_trigger_spo2_reads();												//checked
uint8_t max30102_get_num_available_samples();												//checked
max30102_sample_t* max30102_read_n_samples(uint8_t n_samples, uint8_t *m_success);


#endif /* DRV_MAX30102_H_ */
