#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

//other includes
#include "defs.h"
#include "sensors/EkgSensor.h"
#include "sensors/Spo2Sensor.h"
#include "sensors/TemperatureSensor.h"
#include "bt_com.h"


QueueHandle_t xSensorQueue = NULL;
QueueHandle_t xCommsQueue = NULL;

TimerHandle_t xTemperatureSensorTimer = NULL;
void * pvTemperatureSensorId = NULL;


static Sensor * sensors[3];

void sensors_task(void *pvParameters);
void comms_task(void * pvParameters);

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

    //TODO: ver si las inicilaizaciones de sensores van aca o en la sensor_task

	set_limits(EVENT_TEMPERATURE, LOWEST_TEMPERATURE, HIGHEST_TEMPERATURE);
	set_limits(EVENT_SPO2_SPO2, LOWEST_SPO2, HIGHEST_SPO2);
	set_limits(EVENT_SPO2_BPM, LOWEST_BPM, HIGHEST_BPM);

	BT_com_init();
	sensor_init();

	xCommsQueue = xQueueCreate(UI_SENSOR_QUEUE_LENGTH, sizeof(sensor_event_t));

	xTaskCreate(sensors_task, "sensor task", configMINIMAL_STACK_SIZE + 500, NULL, SENSOR_TASK_PRIORITY, NULL);

	vTaskStartScheduler();
	while (true) ;
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

	sensor_event_t ev;
//	sensors[0]->start_sampling();
//	sensors[1]->start_sampling();
//	sensors[2]->start_sampling();

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
				if(new_range_status == EVENT_RANGE_OVERFLOW ||
						new_range_status == EVENT_RANGE_UNDERFLOW)
					BT_com_set_alarm(ev.type, ev.type < EVENTS_OUT);

		}
	}
}


