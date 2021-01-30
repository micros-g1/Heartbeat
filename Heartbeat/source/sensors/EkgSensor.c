#include "EkgSensor.h"
#include "drv/ad8232/ad8232.h"


static __volatile__ Sensor sensor;

void ekg_init(uint32_t sampling_rate_ms);
void ekg_start_sampling(void);
void ekg_stop_sampling(void);


Sensor * new_ekg_sensor(void)
{
	sensor.type = SENSOR_EKG;
	sensor.init = ekg_init;
	sensor.start_sampling = ekg_start_sampling;
	sensor.stop_sampling = ekg_stop_sampling;
	sensor.status = false;
	return &sensor;
}

void ekg_init(uint32_t sampling_rate_ms)
{
    sensor.status = ad8232_init() == AD8232_SUCCESS;
    ad8232_set_sampling_period(sampling_rate_ms * 1000);
    set_limits(EVENT_EKG, 0.0, 1.0);
}
    

void ekg_start_sampling(void)
{
    sensor.status = ad8232_trigger_reads() == AD8232_SUCCESS;
}

void ekg_stop_sampling(void)
{
	;
}


// ekg ISR
void ADC0_IRQHANDLER(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    float sample = ((float)ad8232_get_new_sample()) / (float)UINT32_MAX;
    sensor.status = write_sample(sample, EVENT_EKG, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
