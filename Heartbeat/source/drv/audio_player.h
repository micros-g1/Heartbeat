/*
 * audio_player.h
 *
 *  Created on: 15 ene. 2021
 *      Author: Tomas
 */

#ifndef DRV_AUDIO_PLAYER_H_
#define DRV_AUDIO_PLAYER_H_
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	AUDIO_PLAYER_SUCCESS = true,
	AUDIO_PLAYER_FAILURE = false
}audio_player_state_t;

audio_player_state_t audio_player_init();

#endif /* DRV_AUDIO_PLAYER_H_ */
