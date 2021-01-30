#pragma once 

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h> 

#include "FreeRTOSConfig.h"
#include "projdefs.h"
#include "portmacro.h"
#include "timers.h"
#include "queue.h"

#define	UI_SENSOR_QUEUE_LENGTH	64

enum {
	EVENT_RANGE_ERROR, EVENT_RANGE_OK, EVENT_RANGE_UNDERFLOW, EVENT_RANGE_OVERFLOW
};

typedef enum {
	SENSOR_EKG, SENSOR_TEMPERATURE, SENSOR_SPO2, N_SENSORS
} sensor_t;

typedef enum {
	EVENT_EKG, EVENT_TEMPERATURE, EVENT_SPO2_EKG, EVENT_SPO2_SPO2, EVENT_SPO2_BPM, N_SENSOR_EVENTS,
	EVENT_EKG_OUT, EVENT_TEMPERATURE_OUT, EVENT_SPO2_EKG_OUT, EVENT_SPO2_SPO2_OUT, EVENT_SPO2_BPM_OUT, EVENTS_OUT,
	EVENT_EKG_IN, EVENT_TEMPERATURE_IN, EVENT_SPO2_EKG_IN, EVENT_SPO2_SPO2_IN, EVENT_SPO2_BPM_IN, EVENTS_IN,
} sensor_event_type_t;


typedef struct {
	sensor_event_type_t type;
	float value;
} sensor_event_t;


#define LOWEST_TEMPERATURE 	35.0
#define HIGHEST_TEMPERATURE 36.5

#define LOWEST_BPM 	60.0
#define HIGHEST_BPM 100.0

#define LOWEST_SPO2		0.95
#define HIGHEST_SPO2	1.00

#define SENSOR_HYSTERESIS 0.025
