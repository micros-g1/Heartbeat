/*
 * bt_com.c
 *
 *  Created on: Jan 30, 2021
 *      Author: rochi
 */
#include "bt_com.h"

bt_com_state_t BT_com_init()
{
	return BT_COM_SUCCESS;
}

bool BT_com_is_connected()
{
	return true;
}

bt_com_state_t BT_com_send_meas(sensor_event_t sens_ev)
{
	return BT_COM_SUCCESS;
}

bt_com_state_t BT_com_set_alarm(sensor_event_type_t source, bool set)
{
	return BT_COM_SUCCESS;
}
