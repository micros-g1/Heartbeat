/*
 * bt_com.c
 *
 *  Created on: Jan 30, 2021
 *      Author: rochi
 */
#include "bt_com.h"
#include "board/peripherals.h"
#include "board/pin_mux.h"

#define HEADER_LEN	1

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
		int i = 0;
		// header
		for (i = 0; i < HEADER_LEN; i++)
			buffer[i] = (uint8_t)'F';
		//tag
		buffer[i++] = BT_com_get_tag(sens_ev.type);
		// length
		buffer[i++] = (uint8_t)sizeof(sens_ev.value);
		// value
		memcpy(&buffer[i], &(sens_ev.value), sizeof(sens_ev.value));
		// UART Transmission to HC05.
		UART_RTOS_Send(&UART3_rtos_handle, buffer, i + sizeof(sens_ev.value));
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
		int i = 0;
		// header
		for (i = 0; i < HEADER_LEN; i++)
			buffer[i] = (uint8_t)'F';
		//tag
		buffer[i++] = (uint8_t)'A';
		buffer[i++] = 2;
		// Message: Source + Set or Reset.
		buffer[i++] = BT_com_get_tag(source);
		buffer[i++] = (uint8_t)(set ? 'S' : 'R');
		// UART Transmission to HC05
		UART_RTOS_Send(&UART3_rtos_handle, buffer, i);
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
			// EKG signal
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
			// Temperature signal
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
