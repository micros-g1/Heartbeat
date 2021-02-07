#ifndef __DEFS_H__
#define __DEFS_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h> 

#include "MK64F12.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "timers.h"
#include "queue.h"
#include "board/peripherals.h"
#include "fsl_debug_console.h"

#define	UI_SENSOR_QUEUE_LENGTH	64

#define EKG_SAMPLING_PERIOD_MS	10
#define TEMP_SAMPLING_PERIOD_MS	1000
#define COMMS_PERIOD_MS			5

//enum {
//	SENSOR_TASK_PRIORITY = tskIDLE_PRIORITY + 1,
//	COMMS_TASK_PRIORITY,
//	SPO2_TASK_PRIORITY
//};

enum {
	COMMS_TASK_PRIORITY = tskIDLE_PRIORITY + 1,
	SPO2_TASK_PRIORITY,
	SENSOR_TASK_PRIORITY
};

enum {
	EVENT_RANGE_ERROR, EVENT_RANGE_OK, EVENT_RANGE_UNDERFLOW, EVENT_RANGE_OVERFLOW
};

typedef enum {
	SENSOR_EKG, SENSOR_TEMPERATURE, SENSOR_SPO2, N_SENSORS
} sensor_t;

typedef enum {
	EVENT_EKG, EVENT_TEMPERATURE, EVENT_SPO2_LED, EVENT_SPO2_SPO2, EVENT_SPO2_BPM, N_SENSOR_EVENTS,
	EVENT_SPO2_SPO2_NOT_VALID, EVENT_SPO2_BPM_NOT_VALID
} sensor_event_type_t;


typedef struct {
	sensor_event_type_t type;
	float value;
} sensor_event_t;


#define LOWEST_TEMPERATURE 	35.0
#define HIGHEST_TEMPERATURE 36.5

#define LOWEST_BPM 	80.0
#define HIGHEST_BPM 100.0

#define LOWEST_SPO2		95
#define HIGHEST_SPO2	100

#define SENSOR_HYSTERESIS 0.025

#endif
