#pragma once

#include "Sensor.h"




class TemperatureSensor : public Sensor
{
public:
    TemperatureSensor(uint32_t sampling_rate_ms);
    ~TemperatureSensor();
    void start_sampling();
	void stop_sampling();

private:
};
