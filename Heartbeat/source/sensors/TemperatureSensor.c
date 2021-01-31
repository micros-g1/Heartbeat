#include "TemperatureSensor.h"
#include "drv/max30205/max30205.h"


static __volatile__ Sensor sensor;
static TimerHandle_t xTimer = NULL;

void read_temp_callback(TimerHandle_t xTimer);

void temperature_start_sampling(void);
void temperature_stop_sampling(void);
void temperature_init(uint32_t sampling_rate_ms);


Sensor * new_temperature_sensor(void)
{
	sensor.type = SENSOR_TEMPERATURE;
	sensor.init = temperature_init;
	sensor.start_sampling = temperature_start_sampling;
	sensor.stop_sampling = temperature_stop_sampling;
	sensor.status = false;

	return &sensor;
}

void temperature_init(uint32_t sampling_rate_ms)
{
	max30205_state_t temperature_sensor_status = max30205_init();
	if (temperature_sensor_status == MAX30205_SUCCESS) {
		xTimer = xTimerCreate(
				"max30205_timer", 		// name for debugging purposes only
				pdMS_TO_TICKS(sampling_rate_ms),	// period
				pdTRUE,					// auto-reload timer (as opposed to one-shot)
				NULL,	            // timer id. it can be NULL if we don't want to use it
				read_temp_callback		// callback
		);
	}

	sensor.status = temperature_sensor_status == MAX30205_SUCCESS && xTimer != NULL;
}

void temperature_stop_sampling(void)
{
	//max30205_shutdown(true);
}
    
void temperature_start_sampling(void)
{
    sensor.status = xTimerStart(xTimer, 0) == pdTRUE;  // if timer can't be started return immediately
}


// temperature sensor timer callback
void read_temp_callback(TimerHandle_t xTimer)
{
	float sample;
	bool status = max30205_temp_read(&sample) == MAX30205_SUCCESS;
	if (status) {
        sensor.status = write_sample(sample, EVENT_TEMPERATURE, NULL);
	}
}
