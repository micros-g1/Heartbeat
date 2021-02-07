/*
 * bt_com.h
 *
 *  Created on: 28 ene. 2021
 *      Author: lisan
 */

#ifndef __BT_COM_H__
#define __BT_COM_H__

#include "defs.h"

typedef enum{
	BT_COM_SUCCESS = true,
	BT_COM_FAILURE = false
} bt_com_state_t;

/*!
 * @brief Initialises BT_com.
 *
 * Initialises the HC05_STATE pin.
 *
 * @return True if initialised successfully.
 */
bt_com_state_t BT_com_init();

/*!
 * @brief Check if the bluetooth a device is connected to the HC05 module
 *
 * @return True is connected, false otherwise.
 */
bool BT_com_is_connected();

/*!
 * @brief Sends a measurement event over BT.
 * 
 * @param sensor_ev : structure containing the mesurement source and value
 *
 * @return BT_COM_SUCCESS if measurement set correctly, BT_COM_FAILURE otherwise
 */
bt_com_state_t BT_com_send_meas(sensor_event_t sens_ev);

/*!
 * @brief Sends a measurement event over BT.
 * 
 * @param source : Source of the alarm
 * @param set : Boolean indicating if the alarm must be SET or RESET
 *
 * @return BT_COM_SUCCESS if alarm set correctly, BT_COM_FAILURE otherwise.
 */
bt_com_state_t BT_com_set_alarm(sensor_event_type_t source, bool set);


/*!
 * @brief Sends error event over BT.
 *
 * @param source : Source of the alarm
 *
 * @return BT_COM_SUCCESS if error was sent correctly, BT_COM_FAILURE otherwise.
 */
bt_com_state_t BT_com_send_error(sensor_event_type_t source);

#endif
