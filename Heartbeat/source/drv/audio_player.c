/*
 * audio_player.c
 *
 *  Created on: 15 ene. 2021
 *      Author: taomasgonzalez, grein
 */

#include <drv/audio_player.h>
#include <drv/uda1380.h>
#include "fsl_debug_console.h"
#include "drivers/fsl_uart.h"
#include <stddef.h>
#include "semphr.h"
#include "task.h"
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

#define AUDIO_PLAYER_QUEUE_LENGTH 10
static volatile audio_player_audio_id_t audio_player_queue[AUDIO_PLAYER_QUEUE_LENGTH];
static volatile int in_pointer = 0;
static volatile int out_pointer = 0;

static TaskHandle_t xTaskAudioPlayer = NULL;
static SemaphoreHandle_t xBinarySemaphore = NULL;

static bool playing = false;

static uint8_t buffer1[AUDIO_PLAYER_DATA_CHUNK] __attribute__((aligned(4)));
static uint8_t buffer2[AUDIO_PLAYER_DATA_CHUNK] __attribute__((aligned(4)));
static uint8_t buffer3[AUDIO_PLAYER_DATA_CHUNK] __attribute__((aligned(4)));

static uint8_t * buffers[AUDIO_PLAYER_N_BUFFERS] = {buffer1, buffer2, buffer3};
static bool buffer_availables[AUDIO_PLAYER_N_BUFFERS] = {true, true, true};
static size_t bytesread[AUDIO_PLAYER_N_BUFFERS] = {0, 0, 0};

static int next_playing = 0;
static int n_used_buffers = 0;

/*callback that is called by uda when a chunk of data
has almost finished playing.*/
static void uda_finished_chunk();

//task that decodes data and sends it to uda1380 to play, chunk by chunk
static void audio_player_task(void *pvParameters);
static void audio_player_set_mp3wrap_audio(audio_player_audio_id_t id);

audio_player_state_t audio_player_init(uint32_t task_priority){
	audio_player_state_t correct_init = AUDIO_PLAYER_SUCCESS;

	//uda related inits
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = uda1380_init() ? AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		uda1380_finished_set_callback(uda_finished_chunk);

	// audio playing tasks and semaphore inits
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = ((xBinarySemaphore = xSemaphoreCreateBinary()) != NULL) ?
				AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = (xTaskCreate(audio_player_task, "audio player task",
						configMINIMAL_STACK_SIZE + 200, NULL,
						task_priority, &xTaskAudioPlayer) == pdTRUE) ?
								AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

	//mp3 decoder init
	if(correct_init == AUDIO_PLAYER_SUCCESS)
		correct_init = mp3wrap_init() ? AUDIO_PLAYER_SUCCESS : AUDIO_PLAYER_FAILURE;

	return correct_init;
}

audio_player_state_t audio_player_play_audio(audio_player_audio_id_t audio_id){
	int out_pointer_minus_one = out_pointer-1;
	if(out_pointer_minus_one < 0)
		out_pointer_minus_one += AUDIO_PLAYER_QUEUE_LENGTH;
	if(in_pointer != out_pointer_minus_one){
		//Add audio to queue
		audio_player_queue[in_pointer++] = audio_id;
		if(in_pointer == AUDIO_PLAYER_QUEUE_LENGTH)
			in_pointer = 0;
		if(!playing){
			//Start playing audio
			xSemaphoreGive(xBinarySemaphore);
		}
		return AUDIO_PLAYER_SUCCESS;
	}
	else
		//Queue is full
		return AUDIO_PLAYER_FAILURE;
}

bool audio_player_currently_playing(){
	return playing;
}

void audio_player_stop_curr_audio(){
	uda1380_stop();
	playing = false;
	in_pointer = out_pointer = 0;
	//reset variables
	for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++){
		buffer_availables[i] = true;
		memset(buffers[i], 0, AUDIO_PLAYER_DATA_CHUNK);
	}
	next_playing = 0;
	n_used_buffers = 0;
}


void audio_player_task(void *pvParameters)
{
	while(true) {
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		//If not playing or queue is not empty
		if(!playing && (in_pointer != out_pointer))
		{
			audio_player_set_mp3wrap_audio(audio_player_queue[out_pointer++]);
			if(out_pointer == AUDIO_PLAYER_QUEUE_LENGTH)
				out_pointer = 0;
			if(!mp3wrap_finished()) // Make sure it's valid
				playing = true;
		}
		//Is playing audio?
		if(playing){

			// if this is the first time the data is being played, fill the first buffer
			if(n_used_buffers == 0){
		    	mp3wrap_decode_next(buffers[0], &bytesread[0]);
				buffer_availables[0] = false;
				n_used_buffers = 1;
			}

			//free one of the buffers to use later on
			int free_buffer_idx = next_playing - 2 + ((next_playing -2 < 0 ) ? AUDIO_PLAYER_N_BUFFERS : 0);
			if(!buffer_availables[free_buffer_idx]){
				buffer_availables[free_buffer_idx] = true;
				n_used_buffers--;
			}

			//check for stop condition (the next buffer from where to play data is empty / available)
		    if(buffer_availables[next_playing]){
		    	for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++)
		    		buffer_availables[i] = true;
		    	next_playing = 0;
		    	n_used_buffers = 0;
		    	playing = false;
		    }
		    else{ // play decoded data and
		    	uda1380_playback(buffers[next_playing], bytesread[next_playing]);
		    	//decode data on available buffer
				for(int i = 0; i < AUDIO_PLAYER_N_BUFFERS; i++){
					if(mp3wrap_finished()) //If finished but still audio in queue
					{
						while(mp3wrap_finished() && in_pointer != out_pointer) //Try to read more audio
						{
							audio_player_set_mp3wrap_audio(audio_player_queue[out_pointer++]);
							if(out_pointer == AUDIO_PLAYER_QUEUE_LENGTH)
								out_pointer = 0;
						}
						//Is there any audio?
						if(mp3wrap_finished())
							break; //Break loop if no
					}
					if(buffer_availables[i]){
						mp3wrap_decode_next(buffers[i], &bytesread[i]);
						buffer_availables[i] = false;
						n_used_buffers++;
					}
				}
				//update next buffer index.
				next_playing = (next_playing ==
						(AUDIO_PLAYER_N_BUFFERS-1)) ? 0 : next_playing + 1;
		    }
		}
	}
}

static void audio_player_set_mp3wrap_audio(audio_player_audio_id_t id)
{
	uint8_t *start_addr;
	size_t len;
	switch(id){
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
}

bool audio_player_is_audio_in_queue(audio_player_audio_id_t id)
{
	int o_p = out_pointer;
	int i_p = in_pointer;
	int i_p_unwrap = i_p + (i_p < o_p ? AUDIO_PLAYER_QUEUE_LENGTH : 0);
	int n = i_p_unwrap - o_p;
	for(int i = 0 ; i < n ; i++)
	{
		int p = o_p + i;
		if(p >= AUDIO_PLAYER_QUEUE_LENGTH)
			p -= AUDIO_PLAYER_QUEUE_LENGTH;
		if(audio_player_queue[p] == id)
			return true;
	}
	return false;
}


//called inside an interrupt
static void uda_finished_chunk(){
	BaseType_t xHigherPriorityTaskWoken;
	// wake up the playing audio task
	xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
