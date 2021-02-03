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

#include <drv/flashmem.h>
#include <drv/uda1380.h>
#include <drv/audio_player.h>
#include "fsl_debug_console.h"
#include "drivers/fsl_uart.h"
#include <stddef.h>
#include "semphr.h"
#include "task.h"


#define AUDIO_PLAYER_DATA_CHUNK 4096
#define AUDIO_PLAYER_N_BUFFERS 2
/*
 * Make sure that your code is not (by mistake) re-writing to the Flash
 * the next time that it runs (without first erasing it) since a second write to a phrase,
 * without first erasing the sector that is in, is an invalid operation that can damage the flash
 * and cause it to behave strangely (often causing hard-faults to occur when the corrupted area is read,
 * where debuggers will tend to display the content as "-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ").
 */

typedef flashmem_file_t audio_player_track_t;

static audio_player_track_t curr_track;
static uint32_t curr_address = (uint32_t)NULL;
static uint32_t curr_remaining_bytes = 0;
static bool playing = false;
static TaskHandle_t xTaskAudioPlayer = NULL;
static SemaphoreHandle_t xBinarySemaphore = NULL;

static uint8_t buffer1[AUDIO_PLAYER_DATA_CHUNK];
static uint8_t buffer2[AUDIO_PLAYER_DATA_CHUNK];
static uint8_t * buffers[AUDIO_PLAYER_N_BUFFERS] = {buffer1, buffer2};
static bool buffer_availables[AUDIO_PLAYER_N_BUFFERS] = {true, true};
static int curr_decompressing = 0;

static void uda_finished_chunk();

void audio_player_task(void *pvParameters);

audio_player_state_t audio_player_init(uint32_t task_priority){
	audio_player_state_t correct_init = AUDIO_PLAYER_SUCCESS;
//	audio_player_state_t correct_init =
//			flashmem_init() == FLASHMEM_SUCCESS ? AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;
//
#ifdef FLASHMEM_PROGRAM
//	if(correct_init == AUDIO_PLAYER_SUCCESS)
//		correct_init = flashmem_program() == FLASHMEM_SUCCESS ?
//				AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

//	uint32_t buff_ram[FLASHMEM_BUFFER_LEN];
//	memcpy(buff_ram, (uint32_t *)FLASHMEM_DEST_ADDR, sizeof(buff_ram));

//	for(int i = 0; i < FLASHMEM_BUFFER_LEN; i++)
//		PRINTF("\r\n%x", (unsigned int) buff_ram[i]);
#endif
#ifdef FLASHMEM_TEST_FLASH
	uint32_t buff_ram[FLASHMEM_BUFFER_LEN];
	while(true){

		memcpy(buff_ram, (uint32_t *) (FLASHMEM_DEST_ADDR), sizeof(buff_ram));

//		for(int i = 0; i < FLASHMEM_BUFFER_LEN; i++){
		UART_WriteBlocking(UART0, buff_ram, 4);
	//		PRINTF("\r\n%x", (unsigned int) buff_ram[i]);
//		}

		memcpy(buff_ram, (uint32_t *) (4096 + FLASHMEM_DEST_ADDR), sizeof(buff_ram));
//		for(int i = 0; i < FLASHMEM_BUFFER_LEN; i++){
		UART_WriteBlocking(UART0, buff_ram, 4);
//			PRINTF("\r\n%x", (unsigned int) buff_ram[i]);
//		}
	}

#endif
//#ifndef FLASHMEM_PROGRAM
#ifndef FLASHMEM_TEST_FLASH

	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = uda1380_init() ? AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		uda1380_finished_set_callback(uda_finished_chunk);
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = ((xBinarySemaphore = xSemaphoreCreateBinary()) != NULL) ?
				AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = (xTaskCreate(audio_player_task, "audio player task",
						configMINIMAL_STACK_SIZE + 166, NULL,
						task_priority, &xTaskAudioPlayer) == pdTRUE) ?
								AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

#endif
//#endif

	return correct_init;
}

audio_player_state_t audio_player_play_audio(audio_player_audio_id_t audio_id){
	if(!playing){
		curr_track = flashmem_get_file((flashmem_file_id_t) audio_id);
		curr_address = curr_track.start_address;
		curr_remaining_bytes = curr_track.length;
		playing = true;
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
	curr_address = (uint32_t) NULL;
	curr_remaining_bytes = 0;
	for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++)
		buffer_availables[i] = true;
	curr_decompressing = 0;
	playing = false;
}


void audio_player_task(void *pvParameters)
{
	uint32_t n_bytes_to_cpy = 0;
	while(true) {
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		if(playing){
			if(curr_remaining_bytes <= 0){
				curr_address = curr_track.start_address;
				curr_remaining_bytes = curr_track.length;
			}
			for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++){
				if(buffer_availables[i] && (curr_remaining_bytes > 0)){
					n_bytes_to_cpy = (curr_remaining_bytes > AUDIO_PLAYER_DATA_CHUNK) ?
							AUDIO_PLAYER_DATA_CHUNK: curr_remaining_bytes;
					memcpy(buffers[i], (uint32_t *)curr_address, n_bytes_to_cpy);
					buffer_availables[i] = false;
					curr_remaining_bytes -= n_bytes_to_cpy;
					curr_address += n_bytes_to_cpy;
				}
			}
//			decompress(buffers[curr_decompressing], decompressed_buffers[curr_decompressing]);
//			uda1380_playback(decompressed_buffers[curr_decompressing], AUDIO_PLAYER_DATA_CHUNK);
			uda1380_playback(buffers[curr_decompressing], AUDIO_PLAYER_DATA_CHUNK);
			curr_decompressing = (curr_decompressing ==
					(AUDIO_PLAYER_N_BUFFERS-1)) ? 0 : curr_decompressing + 1;

		}
	}
}

//called inside an interrupt
static void uda_finished_chunk(){
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	buffer_availables[curr_decompressing] = true;
	xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
