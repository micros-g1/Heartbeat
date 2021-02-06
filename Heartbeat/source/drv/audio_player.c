/*
 * audio_player.c
 *
 *  Created on: 15 ene. 2021
 *      Author: Tomas
 */

/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//https://community.nxp.com/t5/Kinetis-Microcontrollers/How-to-write-my-own-data-to-the-flash-of-K64-above-the-program/td-p/653690

#include <drv/audio_player.h>
#include <drv/uda1380.h>
#include "fsl_debug_console.h"
#include "drivers/fsl_uart.h"
#include <stddef.h>
#include "semphr.h"
#include "task.h"
#include "music.h"
#include "drv/mp3wrap.h"

#define AUDIO_PLAYER_DATA_CHUNK 10000

#define AUDIO_PLAYER_N_BUFFERS 3
#define AUDIO_PLAYER_LEN_ID1 8395
#define AUDIO_PLAYER_LEN_ID2 8827
#define AUDIO_PLAYER_LEN_ID3 8827
#define AUDIO_PLAYER_LEN_ID4 8683
#define AUDIO_PLAYER_LEN_ID5 8107

#define AUDIO_PLAYER_BASE_ADDR (uint8_t *) (0x3e800)
#define AUDIO_PLAYER_ADDR_ID1 AUDIO_PLAYER_BASE_ADDR
#define AUDIO_PLAYER_ADDR_ID2 (AUDIO_PLAYER_ADDR_ID1 + AUDIO_PLAYER_LEN_ID1)
#define AUDIO_PLAYER_ADDR_ID3 (AUDIO_PLAYER_ADDR_ID2 + AUDIO_PLAYER_LEN_ID2)
#define AUDIO_PLAYER_ADDR_ID4 (AUDIO_PLAYER_ADDR_ID3 + AUDIO_PLAYER_LEN_ID3)
#define AUDIO_PLAYER_ADDR_ID5 (AUDIO_PLAYER_ADDR_ID4 + AUDIO_PLAYER_LEN_ID4)
/*
 * Make sure that your code is not (by mistake) re-writing to the Flash
 * the next time that it runs (without first erasing it) since a second write to a phrase,
 * without first erasing the sector that is in, is an invalid operation that can damage the flash
 * and cause it to behave strangely (often causing hard-faults to occur when the corrupted area is read,
 * where debuggers will tend to display the content as "-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ").
 */

static TaskHandle_t xTaskAudioPlayer = NULL;
static SemaphoreHandle_t xBinarySemaphore = NULL;

static bool playing = false;

static uint8_t buffer1[AUDIO_PLAYER_DATA_CHUNK] __attribute__((aligned(4)));
static uint8_t buffer2[AUDIO_PLAYER_DATA_CHUNK] __attribute__((aligned(4)));
static uint8_t buffer3[AUDIO_PLAYER_DATA_CHUNK] __attribute__((aligned(4)));
static uint8_t * buffers[AUDIO_PLAYER_N_BUFFERS] = {buffer1, buffer2, buffer3};
static bool buffer_availables[AUDIO_PLAYER_N_BUFFERS] = {true, true, true};
static int bytesread[AUDIO_PLAYER_N_BUFFERS] = {0, 0, 0};
static int next_playing = 0;
static int n_used_buffers = 0;

static void uda_finished_chunk();
void audio_player_task(void *pvParameters);

audio_player_state_t audio_player_init(uint32_t task_priority){
	audio_player_state_t correct_init = AUDIO_PLAYER_SUCCESS;

	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = uda1380_init() ? AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		uda1380_finished_set_callback(uda_finished_chunk);
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = ((xBinarySemaphore = xSemaphoreCreateBinary()) != NULL) ?
				AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = (xTaskCreate(audio_player_task, "audio player task",
						configMINIMAL_STACK_SIZE + 200, NULL,
						task_priority, &xTaskAudioPlayer) == pdTRUE) ?
								AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = mp3wrap_init() ? AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

	return correct_init;
}

audio_player_state_t audio_player_play_audio(audio_player_audio_id_t audio_id){
	if(!playing){
		playing = true;
		uint8_t *start_addr;
		size_t len;
		switch(audio_id){
		case AUDIO_PLAYER_LOW_HR:
			start_addr = AUDIO_PLAYER_ADDR_ID1;
			len = AUDIO_PLAYER_LEN_ID1;
			break;
		case AUDIO_PLAYER_HIGH_HR:
			start_addr = AUDIO_PLAYER_ADDR_ID2;
			len = AUDIO_PLAYER_LEN_ID2;
			break;
		case AUDIO_PLAYER_LOW_TEMP:
			start_addr = AUDIO_PLAYER_ADDR_ID3;
			len = AUDIO_PLAYER_LEN_ID3;
			break;
		case AUDIO_PLAYER_HIGH_TEMP:
			start_addr = AUDIO_PLAYER_ADDR_ID4;
			len = AUDIO_PLAYER_LEN_ID4;
			break;
		case AUDIO_PLAYER_LOW_SPO2:
			start_addr = AUDIO_PLAYER_ADDR_ID5;
			len = AUDIO_PLAYER_LEN_ID5;
			break;
		default:
			start_addr = NULL;
			len = 0;
			break;
		}
	    mp3wrap_setdata(start_addr, len);
		xSemaphoreGive(xBinarySemaphore);
		return AUDIO_PLAYER_SUCCESS;
	}
	else
		return AUDIO_PLAYER_FAILURE;
}

bool audio_player_currently_playing(){
	return playing;
}

void audio_player_stop_curr_audio(){
	uda1380_stop();
	for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++){
		buffer_availables[i] = true;
		memset(buffers[i], 0, AUDIO_PLAYER_DATA_CHUNK);
	}
	next_playing = 0;
	n_used_buffers = 0;
	playing = false;
}


void audio_player_task(void *pvParameters)
{
	while(true) {
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		if(playing){
			if(n_used_buffers == 0){
		    	mp3wrap_decode_next(buffers[0], &bytesread[0]);
				buffer_availables[0] = false;
				n_used_buffers = 1;
			}
			int free_buffer_idx = next_playing - 2 + ((next_playing -2 < 0 ) ? AUDIO_PLAYER_N_BUFFERS : 0);
			if(!buffer_availables[free_buffer_idx]){
				buffer_availables[free_buffer_idx] = true;
				n_used_buffers--;
			}

		    if(buffer_availables[next_playing]){
		    	for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++)
		    		buffer_availables[i] = true;
		    	next_playing = 0;
		    	n_used_buffers = 0;
		    	playing = false;
		    }
		    else{
		    	uda1380_playback(buffers[next_playing], bytesread[next_playing]);

				for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS && !mp3wrap_finished(); i++){
					if(buffer_availables[i]){
						mp3wrap_decode_next(buffers[i], &bytesread[i]);
						buffer_availables[i] = false;
						n_used_buffers++;
					}
				}

				next_playing = (next_playing ==
						(AUDIO_PLAYER_N_BUFFERS-1)) ? 0 : next_playing + 1;
		    }
		}
	}
}

//called inside an interrupt
static void uda_finished_chunk(){
	BaseType_t xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
