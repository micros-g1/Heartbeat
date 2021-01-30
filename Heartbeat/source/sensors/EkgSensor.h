#pragma once

#include "Sensor.h"



class EkgSensor : public Sensor
{
public:
    EkgSensor();
    ~EkgSensor();
    void start_sampling();
	void stop_sampling();

private:
};
