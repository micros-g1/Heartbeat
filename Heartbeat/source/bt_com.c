/*
 * bt_com.c
 *
 *  Created on: Jan 30, 2021
 *      Author: rochi
 */
#include "bt_com.h"
#include "board/peripherals.h"
#include "board/pin_mux.h"

#define HEADER_LEN	4

uint8_t BT_com_get_tag(sensor_event_type_t event);

static uint8_t buffer[100];

bt_com_state_t BT_com_init(){
	// UART3 already initialized
	// Setup STATE pin
	gpio_pin_config_t output_conf = {
			kGPIO_DigitalInput,
	};
	GPIO_PinInit(BOARD_HC05_STATE_GPIO, BOARD_HC05_STATE_PIN, &output_conf);
	return BT_COM_SUCCESS;
}

bt_com_state_t BT_com_send_meas(sensor_event_t sens_ev){
	sens_ev.value = (float)sens_ev.value;
	bt_com_state_t success = BT_COM_FAILURE;
	if (BT_com_is_connected()){
		int j = 0;
		// header
		for (int i = 0; i < HEADER_LEN; i++){
			buffer[i] = (uint8_t)'F';
			j++;
		}
		//tag
		buffer[j++] = BT_com_get_tag(sens_ev.type);
		// length
		buffer[j++] = (uint8_t)sizeof(sens_ev.value);
		// value
		UART_RTOS_Send(&UART3_rtos_handle, buffer, j);
		uint8_t * temp = (uint8_t *) &sens_ev.value;
		for (int i = 0; i < sizeof(sens_ev.value); i ++){
			UART_RTOS_Send(&UART3_rtos_handle, &temp[i], 1);
		}

		success = BT_COM_SUCCESS;
	}
	else{
		success = BT_COM_FAILURE;
	}
	return success;
}

bt_com_state_t BT_com_set_alarm(sensor_event_type_t source, bool set){

	bt_com_state_t success = BT_COM_FAILURE;
	if(BT_com_is_connected()){
		int j = 0;
		// header
		for (int i = 0; i < HEADER_LEN; i++){
			buffer[i] = (uint8_t)'F';
			j++;
		}
		//tag
		buffer[j++] = (uint8_t)'A';
		buffer[j++] = BT_com_get_tag(source);
		buffer[j++] = (uint8_t)(set ? 'S' : 'R');
		UART_RTOS_Send(&UART3_rtos_handle, buffer, j);
		success = BT_COM_SUCCESS;
	}else{
		success = BT_COM_FAILURE;
	}
	return success;
}

uint8_t BT_com_get_tag(sensor_event_type_t event){
	uint8_t tag;
	switch(event){
		case EVENT_EKG:
			// EKG signal tag
			tag = 'E';
			break;
		case EVENT_SPO2_EKG:
			// Oximeter EKG signal
			tag = 'O';
			break;
		case EVENT_SPO2_BPM:
			// Oximeter pulse signal
			tag = 'P';
			break;
		case EVENT_SPO2_SPO2:
			// Oximeter saturation signal
			tag = 'S';
			break;
		case EVENT_TEMPERATURE:
			tag = 'T';
			break;
		default:
			tag = 0;
			break;
	}
	return tag;
}

bool BT_com_is_connected(){
	bool ret = GPIO_PinRead(BOARD_HC05_STATE_GPIO, BOARD_HC05_STATE_PIN);
	return (bool)ret;
}
