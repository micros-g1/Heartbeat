#include "TemperatureSensor.h"
#include "drv/max30205/max30205.h"


static TemperatureSensor * sensor = nullptr;
static TimerHandle_t xTimer = nullptr;

void read_temp_callback(TimerHandle_t xTimer);

TemperatureSensor::TemperatureSensor(uint32_t sampling_rate_ms) : Sensor(SENSOR_TEMPERATURE, EVENT_TEMPERATURE)
{
    if (sensor != nullptr)
        return;
    
    max30205_state_t temperature_sensor_status = max30205_init();
	if (temperature_sensor_status == MAX30205_SUCCESS) {
		xTimer = xTimerCreate(
				"max30205_timer", 		// name for debugging purposes only
				pdMS_TO_TICKS(sampling_rate_ms),	// period
				pdTRUE,					// auto-reload timer (as opposed to one-shot)
				nullptr,	            // timer id. it can be NULL if we don't want to use it
				read_temp_callback		// callback
		);
    }
	
	status = temperature_sensor_status == MAX30205_SUCCESS && xTimer != nullptr;
    sensor = this;
}

TemperatureSensor::~TemperatureSensor()
{
	stop_sampling();
	max30205_shutdown(true);
}
    
void TemperatureSensor::start_sampling()
{
    status = xTimerStart(xTimer, 0) == pdTRUE;  // if timer can't be started return immediately
}

void TemperatureSensor::stop_sampling()
{
    xTimerStop(xTimer, 0);
}


// temperature sensor timer callback
void read_temp_callback(TimerHandle_t xTimer)
{
	float sample;
	bool status = max30205_temp_read(&sample) == MAX30205_SUCCESS;
	if (status) {
        sensor->status = sensor->write_sample(sample);
	}
}
