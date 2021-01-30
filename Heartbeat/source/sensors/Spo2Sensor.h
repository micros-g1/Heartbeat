#pragma once

#include "Sensor.h"

class Spo2Sensor : public Sensor
{
public:
	Spo2Sensor(uint32_t task_priority);
	~Spo2Sensor();
    void start_sampling();
	void stop_sampling();

private:
};

