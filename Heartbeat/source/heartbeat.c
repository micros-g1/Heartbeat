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


#define alarm_period_ms 10000

QueueHandle_t xSensorQueue = NULL;
QueueHandle_t xCommsQueue = NULL;

TimerHandle_t xAlarmTimer = NULL;

void * pvTemperatureSensorId = NULL;


static Sensor __volatile__ * sensors[3];

static bool correct_init;

static void error_trap();
void play_alarm(sensor_event_t ev);
void sensors_task(void *pvParameters);
void alarm_timer_callback(TimerHandle_t xTimer);

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

	// Set measurements limits to triger alarms
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

	xAlarmTimer = xTimerCreate(
		"audio_alarm_timer", 		// name for debugging purposes only
		pdMS_TO_TICKS(alarm_period_ms),	// period
		pdTRUE,					// auto-reload timer (as opposed to one-shot)
		NULL,	            	// timer id. it can be NULL if we don't want to use it
		alarm_timer_callback					// callback
	);
	if (xAlarmTimer == NULL) {
		PRINTF("Alarm timer could not be initialized!\n");
	}
	vTaskStartScheduler();
	while (true) ;
}

void sensors_task(void *pvParameters)
{
	NVIC_EnableIRQ(PORTB_IRQn);
	xTimerStart(xAlarmTimer, 0);
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
			uint32_t last_range_status = get_range_status(ev.type);
			uint32_t new_range_status = in_range(ev);

			if (new_range_status != last_range_status){
				if(ev.type == EVENT_SPO2_BPM || ev.type == EVENT_SPO2_SPO2 || ev.type == EVENT_TEMPERATURE){
					if(new_range_status == EVENT_RANGE_OVERFLOW || new_range_status == EVENT_RANGE_UNDERFLOW){
						BT_com_set_alarm(ev.type, true);
						xTimerStart(xAlarmTimer,0);
						alarm_timer_callback(xAlarmTimer);
					}
					else if(new_range_status == EVENT_RANGE_OK){
						BT_com_set_alarm(ev.type, false);
					}
				}
			}

//			sensor_event_type_t aux_type = (ev.type == EVENT_SPO2_SPO2_INVALID)? EVENT_SPO2_SPO2 : EVENT_SPO2_BPM;
//			BT_com_set_alarm(aux_type, false);

		}
	}
}

// temperature sensor timer callback
void alarm_timer_callback(TimerHandle_t xTimer)
{
	//Play all alarms
	uint32_t evtype;

	if((evtype = get_range_status(EVENT_SPO2_SPO2)) != EVENT_RANGE_OK){
		if(evtype == EVENT_RANGE_OVERFLOW){
			//UNDEFINED
		}
		if(evtype == EVENT_RANGE_UNDERFLOW){
			audio_player_play_audio(AUDIO_PLAYER_LOW_SPO2);
		}
	}
	if((evtype = get_range_status(EVENT_TEMPERATURE)) != EVENT_RANGE_OK){
		if(evtype == EVENT_RANGE_OVERFLOW){
			audio_player_play_audio(AUDIO_PLAYER_HIGH_TEMP);
		}
		if(evtype == EVENT_RANGE_UNDERFLOW){
			audio_player_play_audio(AUDIO_PLAYER_LOW_TEMP);
		}
	}
	if((evtype = get_range_status(EVENT_SPO2_BPM)) != EVENT_RANGE_OK){
		if(evtype == EVENT_RANGE_OVERFLOW){
			audio_player_play_audio(AUDIO_PLAYER_HIGH_HR);
		}
		if(evtype == EVENT_RANGE_UNDERFLOW){
			audio_player_play_audio(AUDIO_PLAYER_LOW_HR);
		}
	}
}


static void error_trap(){
	while(1){
		;
	}
}
