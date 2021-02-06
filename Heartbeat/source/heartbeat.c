/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    heartbeat.c
 * @brief   Application entry point.
 */

/* Default includes. */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/* other includes. */
#include "drv/audio_player.h"
#include "libs/helix/pub/mp3dec.h"
//#include "mp3_sample.h"

/*******************************************************************************
 * TEST SIGNAL
 ******************************************************************************/


//other includes
#include "defs.h"
#include "sensors/EkgSensor.h"
#include "sensors/Spo2Sensor.h"
#include "sensors/TemperatureSensor.h"
#include "drv/audio_player.h"
#include "bt_com.h"


QueueHandle_t xSensorQueue = NULL;
QueueHandle_t xCommsQueue = NULL;

TimerHandle_t xTemperatureSensorTimer = NULL;
void * pvTemperatureSensorId = NULL;


static Sensor * sensors[3];
static bool alarm_set = false;
static bool correct_init;

static void error_trap();
void handle_alarms(sensor_event_t ev);
void sensors_task(void *pvParameters);

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * @brief   Application entry point.
 */


int main(void)
{

	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	    /* Init FSL debug console. */
	BOARD_InitDebugConsole();
	#endif

    NVIC_SetPriority(I2C0_IRQn, 4);
    NVIC_SetPriority(PORTB_IRQn, 4);
    NVIC_SetPriority(ADC0_IRQn, 4);
    NVIC_SetPriority(UART3_RX_TX_IRQn, 5);
	NVIC_SetPriority(I2S0_Tx_IRQn, 5);
    //TODO: ver si las inicilaizaciones de sensores van aca o en la sensor_task

	set_limits(EVENT_TEMPERATURE, LOWEST_TEMPERATURE, HIGHEST_TEMPERATURE);
	set_limits(EVENT_SPO2_SPO2, LOWEST_SPO2, HIGHEST_SPO2);
	set_limits(EVENT_SPO2_BPM, LOWEST_BPM, HIGHEST_BPM);

	correct_init = true;
	if(correct_init)
		correct_init = BT_com_init() == BT_COM_SUCCESS;

	if(correct_init)
		sensor_init();


	if(!correct_init)
		error_trap();

	xCommsQueue = xQueueCreate(UI_SENSOR_QUEUE_LENGTH, sizeof(sensor_event_t));

	xTaskCreate(sensors_task, "sensor task", configMINIMAL_STACK_SIZE + 166, NULL, SENSOR_TASK_PRIORITY, NULL);

	vTaskStartScheduler();
	while (true) ;
}


void handle_alarms(sensor_event_t ev){
	if(!alarm_set) return;

	audio_player_audio_id_t audio_id;

	switch(ev.type){
	case EVENT_SPO2_BPM:
		if(ev.value < LOWEST_BPM)
			audio_id = AUDIO_PLAYER_LOW_HR;
		else if(ev.value > HIGHEST_BPM)
			audio_id= AUDIO_PLAYER_HIGH_HR;
		break;
	case EVENT_SPO2_SPO2:
		if(ev.value < LOWEST_SPO2)
			audio_id = AUDIO_PLAYER_LOW_SPO2;
		break;
	case EVENT_TEMPERATURE:
		if(ev.value < LOWEST_TEMPERATURE)
			audio_id = AUDIO_PLAYER_LOW_TEMP;
		else if(ev.value < HIGHEST_TEMPERATURE)
			audio_id = AUDIO_PLAYER_HIGH_TEMP;
		break;
	default:
		audio_id = AUDIO_PLAYER_N_AUDIOS;
		break;
	}

	if(!audio_player_currently_playing())
		audio_player_play_audio(audio_id);
}
void sensors_task(void *pvParameters)
{
	NVIC_EnableIRQ(PORTB_IRQn);

	sensors[0] = new_temperature_sensor();
	sensors[0]->init(TEMP_SAMPLING_PERIOD_MS);

	sensors[1] = new_spo2_sensor();
	sensors[1]->init(SPO2_TASK_PRIORITY);

	sensors[2] = new_ekg_sensor();
	sensors[2]->init(EKG_SAMPLING_PERIOD_MS);


	if(audio_player_init(5) != AUDIO_PLAYER_SUCCESS)
		error_trap();

	sensor_event_t ev;
	for (int i = 0; i < N_SENSORS; i++) {
		sensors[i]->start_sampling();
	}

	while (true) {
		if (read_sample(&ev)) {
			BT_com_send_meas(ev);

//			uint32_t last_range_status = get_range_status(ev.type);
			uint32_t new_range_status = in_range(ev);

//			if (new_range_status != last_range_status) {
//				ev.type = new_range_status == EVENT_RANGE_OK ? ev.type+N_SENSOR_EVENTS+1 : ev.type+EVENTS_OUT+1;
//				if (ev.type >= N_SENSOR_EVENTS && ev.type < EVENTS_IN)
			if(ev.type == EVENT_SPO2_BPM || ev.type == EVENT_SPO2_SPO2){
				if(new_range_status == EVENT_RANGE_OVERFLOW || new_range_status == EVENT_RANGE_UNDERFLOW){
					alarm_set = true;
					BT_com_set_alarm(ev.type, ev.type < EVENTS_OUT);
				}
				else
					alarm_set = false;
			}
			handle_alarms(ev);
		}
	}
}

static void error_trap(){
	while(1){
		;
	}
}
