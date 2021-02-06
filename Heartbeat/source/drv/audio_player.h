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

typedef enum
{
	AUDIO_PLAYER_LOW_HR,
	AUDIO_PLAYER_HIGH_HR,
	AUDIO_PLAYER_LOW_TEMP,
	AUDIO_PLAYER_HIGH_TEMP,
	AUDIO_PLAYER_LOW_SPO2,
	AUDIO_PLAYER_N_AUDIOS
}audio_player_audio_id_t;

audio_player_state_t audio_player_init(uint32_t task_priority);
/*can't play audio without having previously stopped
the currently playing audio (if such exists) with audio_player_stop_curr_audio*/
audio_player_state_t audio_player_play_audio(audio_player_audio_id_t audio_id);
bool audio_player_currently_playing();
void audio_player_stop_curr_audio();

#endif /* DRV_AUDIO_PLAYER_H_ */
