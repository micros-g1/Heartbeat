/*
 * audio_player.h
 *
 *  Created on: 15 ene. 2021
 *      Author: taomasgonzalez
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

/**
 * @brief Initialises AUDIO PLAYER
 *
 * @param task_priority: priority of the task that plays audio.
 *
 * @return status indicating if operation was successful
 */
audio_player_state_t audio_player_init(uint32_t task_priority);

/**
 * @brief Initialises AUDIO PLAYER
 *
 * Can not play audio without having previously stopped
 * the currently playing audio (if such audio exists) with audio_player_stop_curr_audio.
 *
 * @param audio_id: ID of the audio to be played.
 *
 * @return status indicating if operation was successful.
 */
audio_player_state_t audio_player_play_audio(audio_player_audio_id_t audio_id);
/**
 * @brief AUDIO PLAYER CURRENTLY PLAYING
 *
 * @return true if the AUDIO PLAYER is currently playing audio. false otherwise.
 */
bool audio_player_currently_playing();
/**
 * @brief AUDIO PLAYER STOP CURRENT AUDIO.
 *
 * This function should be called before changing an
 *  audio that is currently being played.
 */
void audio_player_stop_curr_audio();

#endif /* DRV_AUDIO_PLAYER_H_ */
