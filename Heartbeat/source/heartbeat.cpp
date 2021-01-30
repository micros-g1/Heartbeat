#include "defs.h"
#include "MK64F12.h"
#include "sensors/EkgSensor.h"
#include "sensors/Spo2Sensor.h"
#include "sensors/TemperatureSensor.h"
#include "bt_com.h"

typedef union {
	struct {
		uint8_t temperature_sensor : 1;
		uint8_t temperature_timer : 1;
		uint8_t ekg : 1;
		uint8_t spo2 : 1;
		uint8_t sensor_queue : 1;
	};
	uint8_t as_int;
} heartbeat_status_t;


__volatile__ QueueHandle_t xSensorQueue = NULL;
__volatile__ QueueHandle_t xCommsQueue = NULL;

TimerHandle_t xTemperatureSensorTimer = NULL;
void * pvTemperatureSensorId = NULL;

__volatile__ heartbeat_status_t status;

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

	sensors[0] = new TemperatureSensor(1000);
	sensors[1] = new EkgSensor();
	sensors[2] = new Spo2Sensor(2);

	Sensor::set_limits(EVENT_TEMPERATURE, LOWEST_TEMPERATURE, HIGHEST_TEMPERATURE);
	Sensor::set_limits(EVENT_SPO2_SPO2, LOWEST_SPO2, HIGHEST_SPO2);
	Sensor::set_limits(EVENT_SPO2_BPM, LOWEST_BPM, HIGHEST_BPM);

    NVIC_SetPriority(I2C_A_IRQn, 5);
    NVIC_SetPriority(PORTB_IRQn, 4);
	NVIC_EnableIRQ(PORTB_IRQn);

	BT_com_init();

	xTaskCreate(sensor_task, "sensor task", configMINIMAL_STACK_SIZE + 166, nullptr, tskIDLE_PRIORITY+2, nullptr);
	xTaskCreate(comms_task, "comms task", configMINIMAL_STACK_SIZE + 166, nullptr, tskIDLE_PRIORITY+1, nullptr);

	vTaskStartScheduler();
	for (;;)
		;
}

void sensors_task(void *pvParameters)
{
	sensor_event_t ev;
	while (true) {
		if (Sensor::read_sample(&ev)) {
			xQueueSend(xCommsQueue, &ev, pdMS_TO_TICKS(100));

			uint32_t last_range_status = Sensor::get_range_status(ev.type);
			uint32_t new_range_status = Sensor::in_range(ev);

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
		if (xQueueReceive(xCommsQueue, &ev, 0) == pdFAIL)
			continue;

		if (ev.type < N_SENSOR_EVENTS)
			BT_com_send_meas(ev);
		else if (ev.type < EVENTS_IN)
			BT_com_set_alarm(ev.type, ev.type < EVENTS_OUT);
	}
}

