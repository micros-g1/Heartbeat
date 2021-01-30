#pragma once

#include "Sensor.h"



class EkgSensor : public Sensor
{
public:
    EkgSensor(uint32_t sampling_rate_ms);
    ~EkgSensor();
    void start_sampling();
	void stop_sampling();

private:
};
