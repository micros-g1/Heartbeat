/*
 * ad8232.h
 *
 *  Created on: 13 ene. 2021
 *      Author: taomasgonzalez
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

/**
 * @brief Initialises AD8232
 *
 * @return status indicating if operation was successful
 */
ad8232_state_t ad8232_init();

/**
 * @brief Set AD8232 sampling period
 *
 * @param micro_segs : new period in microseconds
 *
 * @return status indicating if operation was successful
 */
ad8232_state_t ad8232_set_sampling_period(uint64_t micro_segs);

/**
 * @brief Triggers AD8232 readings (starts sampling)
 *
 * @return status indicating if operation was successful
 */
ad8232_state_t ad8232_trigger_reads();

/**
 * @brief Stops AD8232 readings (stops sampling)
 *
 * @return status indicating if operation was successful
 */
ad8232_state_t ad8232_stop_reading();

/**
 * @brief Get new samples
 *
 * @return value of AD8232 sample
 */
uint32_t ad8232_get_new_sample();

#endif /* DRV_AD8232_H_ */
