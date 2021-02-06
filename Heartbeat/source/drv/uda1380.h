/*
 * uda1380.h
 *
 *  Created on: 31 Jan 2021
 *      Author: grein
 */

#ifndef DRV_UDA1380_H_
#define DRV_UDA1380_H_

#include <stddef.h>
#include <drv/uda1380_hw.h>

/*----------------------------------------------------------------
 * UDA1380 Driver
 * Setup configuration is defined in uda1380_hw.h
 *---------------------------------------------------------------*/

typedef void (*uda1380_callback_t)(void);

/*!
 * @brief Initialises the UDA1380.
 *
 * Initialises the UDA1380 peripheral.
 *
 * @return True if initialised successfully.
 */

bool uda1380_init();

/*!
 * @brief Sends I2S Audio data to UDA1380
 *
 * @param data : pointer to audio data
 * @param data_length : length of audio data
 */
void uda1380_playback(const uint8_t* data, size_t data_length);

/*!
 * @brief Stops playing data
 */
void uda1380_stop();

/*!
 * @brief Set finished playing callback
 *
 * @param callback : pointer to callback function
 */
void uda1380_finished_set_callback(uda1380_callback_t callback);

#endif /* DRV_UDA1380_H_ */
