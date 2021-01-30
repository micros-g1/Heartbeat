/*
 * Sensor.h
 *
 *  Created on: Jan 23, 2021
 *      Author: rochi
 */

#pragma once 
#include "defs.h"



class Sensor
{
public:
	Sensor(sensor_t sensor_type, sensor_event_type_t default_event_type);
	~Sensor();
	virtual void start_sampling() = 0;
	virtual void stop_sampling() = 0;
	static bool read_sample(sensor_event_t * event);
	bool write_sample(float sample, sensor_event_type_t event_type=N_SENSOR_EVENTS, BaseType_t * pxHigherPriorityTaskWoken=nullptr);

	const sensor_t type;
	const sensor_event_type_t default_event_type;


	static void set_limits(sensor_event_type_t ev, float min, float max);
	static uint32_t get_range_status(sensor_event_type_t ev);
	static uint32_t in_range(sensor_event_t ev);
	bool status;

private:
	static QueueHandle_t xSensorQueue;
	static float min_values[N_SENSOR_EVENTS];
	static float max_values[N_SENSOR_EVENTS];

	static float threshold_low[N_SENSOR_EVENTS];
	static float threshold_high[N_SENSOR_EVENTS];

	static uint32_t range_status[N_SENSOR_EVENTS];
};
