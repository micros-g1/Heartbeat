/*
 * uda1380.h
 *
 *  Created on: 31 Jan 2021
 *      Author: grein
 */

#ifndef DRV_UDA1380_H_
#define DRV_UDA1380_H_

#include <stddef.h>

typedef void (*uda1380_callback_t)(void);

bool uda1380_init();
void uda1380_playback(const uint8_t* data, size_t data_length);
void uda1380_stop();
void uda1380_finished_set_callback(uda1380_callback_t callback);

#endif /* DRV_UDA1380_H_ */
