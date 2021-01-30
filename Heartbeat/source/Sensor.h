#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "defs.h"



typedef struct {
	sensor_t type;
	void (* init)(uint32_t);
	void (* start_sampling)();
	void (* stop_sampling)();
	bool status;
} Sensor;

void sensor_init();

bool write_sample(float sample, sensor_event_type_t event_type, BaseType_t * pxHigherPriorityTaskWoken);
bool read_sample(sensor_event_t * event);

void set_limits(sensor_event_type_t ev, float min, float max);
uint32_t get_range_status(sensor_event_type_t ev);
uint32_t in_range(sensor_event_t ev);



#endif
