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

/**
 * @brief Initialises MAX30102
 * @param initial_mode : specifies mode for initialisation
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_init(max30102_mode_t initial_mode);

/**
 * @brief Request rev ID
 * Queries MAX30102 and reads rev ID
 * @param rev_id : pointer where to save rev ID
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_get_revision_id(uint8_t* rev_id);

/**
 * @brief Request part ID
 * Queries MAX30102 and reads part ID
 * @param part_id : pointer where to save part ID
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_get_part_id(uint8_t* part_id);

/**
 * @brief Enables/Disables temperature ready interrupt
 * @param die_tmp_rdy_en : true if should be enabled, false if should be disabled
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_die_temp_rdy_en(bool die_tmp_rdy_en);

/**
 * @brief Enables/Disables ambient light cancellation overflow interrupt
 * @param alc_ovf_en : true if should be enabled, false if should be disabled
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_alc_ovf_en(bool alc_ovf_en);

/**
 * @brief Enables/Disables next FIFO data ready interrupt
 * @param ppg_rdy_en : true if should be enabled, false if should be disabled
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_ppg_rdy_en(bool ppg_rdy_en);

/**
 * @brief Enables/Disables FIFO almost full interrupt
 * @param a_full_en : true if should be enabled, false if should be disabled
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_a_full_en(bool a_full_en);

/**
 * @brief Requests interrupt status
 * @param status : pointer to interrupt status structure where to store flag status
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_get_interrupt_status(max30102_interrupt_status_t *status);

/**
 * @brief Sets config mode
 * @param config : pointer to config mode to set
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_mode_config(const max30102_mode_configuration_t *config);

/**
 * @brief Sets mode
 * @param conf : mode to set
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_mode(max30102_mode_t conf);

/**
 * @brief Enable/Disable shutdown (power saving mode)
 * @param shdn : True if should enable shutdown, false if should disable shutdown
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_shdn(bool shdn);

/**
 * @brief Reset MAX30102
 * @param reset : True if should reset.
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_reset(bool reset);

/**
 * @brief sets MAX30102 FIFO Config
 * @param config : pointer to struct with FIFO config to set
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_fifo_config(const max30102_fifo_configuration_t *config);

/**
 * @brief Sets number of free spaces remaining for FIFO Almost full interrupt
 * @param fifo_a_full : value to write
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_fifo_a_full(uint8_t fifo_a_full);

/**
 * @brief Enables/Disables FIFO Rollover
 * @param roll_over_en : True if should enable FIFO Rollover, false if should disable
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_fifo_roll_over_en(bool roll_over_en);

/**
 * @brief Sets number of samples for averaging data in FIFO
 * @param smp_ave : Configuration indicating number of samples to average
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_fifo_smp_ave(max30102_smp_ave_t smp_ave);

/**
 * @brief Configures LED Current
 * @param curr_lvl : Value indicating current
 * @param led_addr : Address of led config register, used for selecting which led to configure
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_led_current(uint8_t curr_lvl,  max30102_addr_t led_addr);

/**
 * @brief Triggers read temperature
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_trigger_temp_read();

/**
 * @brief Checks if temperature measurement is ready
 * @param rdy : pointer of bool variable where to save flag
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_is_temp_read_ready(bool* rdy);

/**
 * @brief Blocking function, wait until temperature measurement is ready
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_wait_temp_read_ready();

/**
 * @brief Gets temperature measurement, in degree Celsius
 * @param temp : pointer to value where to store temperature
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_get_temperature_c(float* temp);

/**
 * @brief Sets SPO2 Config
 * @param config : pointer to value indicating configuration
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_spo2_config(const max30102_spo2_configuration_t *config);

/**
 * @brief Sets MAX30102 SPO2 ADC range
 * @param spo2_adc_rge : value indicating ADC range
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_spo2_adc_rge(max30102_spo2_adc_resolution_t spo2_adc_rge);

/**
 * @brief Sets MAX30102 SPO2 sampling rate
 * @param spo2_sample_rate : Value indicating sampling rate to use
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_sr(max30102_spo2_sample_rate_t spo2_sample_rate);

/**
 * @brief Sets LEDs pulse width
 * @param led_pw : Value indicating pulse width
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_set_led_pw(max30102_led_pw_t led_pw);

/**
 * @brief Triggers MAX30102 SPO2 readings (start sampling)
 *
 * @return status indicating if operation was successful
 */
max30102_state_t max30102_trigger_spo2_reads();

/**
 * @brief Read MAX30102 samples
 *
 * @param ir_data : pointer to variable where to store IR measurement data
 * @param red_data : pointer to variable where to store RED measurement data
 *
 * @return status indicating if operation was successful
 */
uint8_t max30102_read_sample(uint32_t *ir_data, uint32_t *red_data);


#endif /* DRV_MAX30102_H_ */
