#include "EkgSensor.h"
#include "drv/ad8232/ad8232.h"


static EkgSensor * sensor = nullptr;


EkgSensor::EkgSensor() : Sensor(SENSOR_EKG, EVENT_EKG)
{
    if (sensor != nullptr)
        return;

    status = ad8232_init() == AD8232_SUCCESS;
    Sensor::set_limits(EVENT_EKG, 0.0, 1.0);
}
    
EkgSensor::~EkgSensor()
{
	stop_sampling();
}


void EkgSensor::start_sampling()
{
    status == ad8232_trigger_reads() == AD8232_SUCCESS;
}

void EkgSensor::stop_sampling()
{
}




// ekg ISR
void ADC0_IRQHANDLER(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    float sample = float(ad8232_get_new_sample()) / float(UINT32_MAX);
    sensor->status = sensor->write_sample(sample, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
