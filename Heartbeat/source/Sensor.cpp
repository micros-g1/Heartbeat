/*
 * Sensor.cpp
 *
 *  Created on: Jan 23, 2021
 *      Author: rochi
 */


#include "Sensor.h"


static uint32_t sensor_count = 0;
static QueueHandle_t Sensor::xSensorQueue = nullptr;

Sensor::Sensor(sensor_t sensor_type, sensor_event_type_t default_event_type)
: type(sensor_type), default_event_type(default_event_type)
{
	if (xSensorQueue == nullptr) {
		xSensorQueue = xQueueCreate(UI_SENSOR_QUEUE_LENGTH, sizeof(sensor_event_t));

		memset(range_status, EVENT_RANGE_ERROR, N_SENSOR_EVENTS);
	}
	sensor_count++;
}

Sensor::~Sensor()
{
	if (--sensor_count == 0 && xSensorQueue != nullptr) {
		vQueueDelete(xSensorQueue);
		xSensorQueue = nullptr;
	}
}

bool Sensor::read_sample(sensor_event_t * event)
{
	return xQueueReceive(xSensorQueue, (void * const) event, portMAX_DELAY) == pdTRUE;
}

bool Sensor::write_sample(float sample, sensor_event_type_t event_type=N_SENSOR_EVENTS, BaseType_t * pxHigherPriorityTaskWoken=nullptr)
{
	if (xSensorQueue == nullptr) 
		return false;

	sensor_event_t event = {
		.type = event_type == N_SENSOR_EVENTS ? default_event_type : event_type,
		.value = sample
	};

	BaseType_t success = pdFALSE; 
	if (pxHigherPriorityTaskWoken == nullptr) {
		success = xQueueSendToBack(xSensorQueue, &event, 0);
	} else {
		success = xQueueSendToBackFromISR(xSensorQueue, &event, pxHigherPriorityTaskWoken);
	}

	return success == pdTRUE;
}

void Sensor::set_limits(sensor_event_t ev, float min, float max)
{
	if (ev < N_SENSOR_EVENTS) {
		min_values[ev] = min;
		max_values[ev] = max;

		threshold_low[ev] = min + (max-min)*SENSOR_HYSTERESIS;
		threshold_high[ev] = max - (max-min)*SENSOR_HYSTERESIS;

		range_status[ev] = EVENT_RANGE_OK;
	}
}

uint32_t Sensor::in_range(sensor_event_t ev)
{
	if (ev.type < N_SENSOR_EVENTS && range_status[ev.type] != EVENT_RANGE_ERROR) {

		float min = min_values[ev.type];
		float max = max_values[ev.type];

		if (range_status[ev.type] == EVENT_RANGE_UNDERFLOW)
			min = threshold_low[ev];
		else if (range_status[ev.type] == EVENT_RANGE_OVERFLOW)
			max = threshold_high[ev];

		if (ev.value > max)
			range_status[ev.type] = EVENT_RANGE_OVERFLOW;
		else if (ev.value < min)
			range_status[ev.type] = EVENT_RANGE_UNDERFLOW;
		else
			range_status[ev.type] = EVENT_RANGE_OK;
	}

	return range_status[ev.type];
}

uint32_t Sensor::get_range_status(sensor_event_type_t ev)
{
	return ev < N_SENSOR_EVENTS ? range_status[ev] : EVENT_RANGE_ERROR;
}
