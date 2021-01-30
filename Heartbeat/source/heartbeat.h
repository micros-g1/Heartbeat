/*
 * heartbeat.h
 *
 *  Created on: Jan 17, 2021
 *      Author: rochi
 */

#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#include "defs.h"


typedef union {
	struct {
		uint8_t temperature_sensor : 1;
		uint8_t temperature_timer : 1;
		uint8_t ekg : 1;
		uint8_t spo2 : 1;
		uint8_t sensor_queue : 1;
	};
	uint8_t as_int;
} heartbeat_status_t;


bool heartbeat_init();

void hearbeat_deinit();

void heartbeat_mainloop();

bool heartbeat_ok();
heartbeat_status_t heartbeat_status();

#endif /* HEARTBEAT_H_ */
