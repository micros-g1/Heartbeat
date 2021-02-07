/*
 * max30205.h
 *
 *  Created on: 13 ene. 2021
 *      Author: taomasgonzalez
 */

#ifndef DRV_MAX30205_H_
#define DRV_MAX30205_H_
#include <stdbool.h>
#include <stdint.h>
#include "max30205_registers.h"

typedef enum
{
	MAX30205_SUCCESS = true,
	MAX30205_FAILURE = false
}max30205_state_t;


/**
 * @brief Initialises MAX30205
 * Power-On Reset Value:
 * These default POR values correspond to the following modes of operation:
 	 * Comparator mode
 	 * OS active low
 	 * 1 fault, fault queue
	 * Normal data format
	 * Timeout enabled for MAX30205
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_init();

/**
 * @brief Sets MAX30205 configuration
 * @param config : pointer to MAX30205 configuration to set
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_config(const max30205_config_t *config);
/**
 * @brief MAX30205 Set One Shot
 *
 * When in shutdown mode (and continuos conversions are not necessary),
 * Setting One Shot begins a temperature conversion and
 * returns to shutdown mode when the conversion is finished
 *
 * @param one_shot : true for setting the one shot bit (beginning the conversion)
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_one_shot(bool one_shot);
/**
 * @brief MAX30205 Set Timeout value
 *
 * @param not_timeout : true to disable bus timeout.
 * false to disable the i2c compatible interface when SDA is low for more than 50ms.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_not_timeout(bool not_timeout);
/**
 * @brief MAX30205 Set Fault Queue
 *
 * @param fault_queue : determine the number of faults necessary to trigger an OS condition
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_fault_queue(max30205_fault_queue_t fault_queue);
/**
 * @brief MAX30205 Set Overtemperature Shutdown (OS) Polarity
 * OS is an open-drain output under all conditions and requires
 * a pullup resistor to output a high voltage.
 *
 * @param os_polarity : false to force the OS output polarity to active low.
 * true to set the OS output polarity to active high.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_os_polarity(bool os_polarity);
/**
 * @brief MAX30205 Set Comparator / Interrupt mode
 * In comparator mode, OS is asserted when the temperature rises above the
 * TOS value. OS is deasserted when the temperature drops below the THYST value.
 * In interrupt mode, exceeding TOS also asserts OS.
 * OS remains asserted until a read operation is performed on any of the registers.
 *
 * @param not_comparator_interrupt : false 0 to operate OS in comparator mode
 * true to operate OS in interrupt mode.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_not_comparator_interrupt(bool not_comparator_interrupt);
/**
 * @brief MAX30205 Set Shutdown value
 *
 * Places the device on shutdown mode
 * (or not, depending on shutdown value)
 *
 * @param shutdown : true for placing the device on shutdown mode.
 * false to get the device out of shutdown mode.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_shutdown(bool shutdown);

/**
 * @brief Sets MAX30205 temperature hysteresis
 * @param float : new hysteresis value
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_thyst(float thyst);
/**
 * @brief Gets MAX30205 temperature hysteresis
 * @param float : pointer to float where the current
 * hysteresis value will be stored.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_get_thyst(float *thyst);

/**
 * @brief Sets MAX30205 overtemperature shutdown value
 * @param tos : new overtemperature shutdown value
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_set_tos(float tos);
/**
 * @brief Gets MAX30205 overtemperature shutdown
 * @param tos : pointer to variable where the current
 * overtemperature shutdown value will be stored.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_get_tos(float *tos);

/**
 * @brief MAX30205 get temperature reading
 * @param temp : pointer to variable where the current
 * temperature value will be stored.
 *
 * @return status indicating if operation was successful
 */
max30205_state_t max30205_temp_read(float *temp);

#endif /* DRV_MAX30205_H_ */
