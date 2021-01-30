#ifndef __BT_COM_H__
#define __BT_COM_H__

#include "defs.h"

typedef enum{
	BT_COM_SUCCESS = true,
	BT_COM_FAILURE = false
} bt_com_state_t;

bt_com_state_t BT_com_init();
bool BT_com_is_connected();
bt_com_state_t BT_com_send_meas(sensor_event_t sens_ev);
bt_com_state_t BT_com_set_alarm(sensor_event_type_t source, bool set);

#endif
