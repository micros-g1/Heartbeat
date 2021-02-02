#include "defs.h"
#include "board.h"
#include "peripherals.h"
#include "sensors/EkgSensor.h"
#include "sensors/Spo2Sensor.h"
#include "sensors/TemperatureSensor.h"
#include "bt_com.h"
#include "fsl_debug_console.h"

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

//    NVIC_SetPriority(I2C0_IRQn, 4);
//    NVIC_SetPriority(PORTB_IRQn, 4);

    NVIC_SetPriority(ADC0_IRQn, 4);
    NVIC_SetPriority(UART3_RX_TX_IRQn, 5);

    //TODO: ver si las inicilaizaciones de sensores van aca o en la sensor_task

	set_limits(EVENT_TEMPERATURE, LOWEST_TEMPERATURE, HIGHEST_TEMPERATURE);
	set_limits(EVENT_SPO2_SPO2, LOWEST_SPO2, HIGHEST_SPO2);
	set_limits(EVENT_SPO2_BPM, LOWEST_BPM, HIGHEST_BPM);

	BT_com_init();
	sensor_init();

	xCommsQueue = xQueueCreate(UI_SENSOR_QUEUE_LENGTH, sizeof(sensor_event_t));

	xTaskCreate(sensors_task, "sensor task", configMINIMAL_STACK_SIZE + 166, NULL, SENSOR_TASK_PRIORITY, NULL);
	xTaskCreate(comms_task, "comms task", configMINIMAL_STACK_SIZE + 166, NULL, COMMS_TASK_PRIORITY, NULL);

	vTaskStartScheduler();
	while (true) ;
}

void sensors_task(void *pvParameters)
{
//	NVIC_EnableIRQ(PORTB_IRQn);

//	sensors[0] = new_temperature_sensor();
//	sensors[0]->init(TEMP_SAMPLING_PERIOD_MS);
//
//	sensors[1] = new_spo2_sensor();
//	sensors[1]->init(SPO2_TASK_PRIORITY);

	sensors[2] = new_ekg_sensor();
	sensors[2]->init(EKG_SAMPLING_PERIOD_MS);

//
	sensor_event_t ev;
	sensors[2]->start_sampling();
//	for (unsigned int i = 0; i < N_SENSORS; i++) {
//		sensors[i]->start_sampling();
//	}

	while (true) {
		if (read_sample(&ev)) {
			xQueueSend(xCommsQueue, &ev, pdMS_TO_TICKS(100));

			uint32_t last_range_status = get_range_status(ev.type);
			uint32_t new_range_status = in_range(ev);

			if (new_range_status != last_range_status) {
				ev.type = new_range_status == EVENT_RANGE_OK ? ev.type+N_SENSOR_EVENTS+1 : ev.type+EVENTS_OUT+1;
				xQueueSend(xCommsQueue, &ev, pdMS_TO_TICKS(100));
			}
		}
	}
}

void comms_task(void * pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	sensor_event_t ev;
	while (true) {
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
		while (xQueueReceive(xCommsQueue, &ev, 0) == pdPASS) {
			if (ev.type < N_SENSOR_EVENTS)
				BT_com_send_meas(ev);
			else if (ev.type < EVENTS_IN)
				BT_com_set_alarm(ev.type, ev.type < EVENTS_OUT);
		}
	}
}

