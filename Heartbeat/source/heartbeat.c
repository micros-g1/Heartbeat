/*
 * Copyright 2016-2020 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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
 
/**
 * @file    heartbeat.c
 * @brief   Application entry point.
 */

/* Default includes. */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"

/* other includes. */
#include "drv/audio_player.h"
#include "drv/flashmem.h"
/*******************************************************************************
 * TEST SIGNAL
 ******************************************************************************/


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_A_IRQn I2C0_IRQn

/* Priorities at which the tasks are created.  */
#define mainEXAMPLE_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Application API */
static void example_task(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * @brief   Application entry point.
 */


int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    if(flashmem_init() == FLASHMEM_SUCCESS)
    	if(flashmem_program() == FLASHMEM_SUCCESS)
    		PRINTF("INIT CORRECT\r\n");

    NVIC_SetPriority(I2C_A_IRQn, 5);
	NVIC_SetPriority(I2S0_Tx_IRQn, 5);

	/* RTOS Init Tasks. */
	if (xTaskCreate(example_task, "example_task",
	configMINIMAL_STACK_SIZE + 166, NULL, mainEXAMPLE_TASK_PRIORITY, NULL) != pdPASS) {
		PRINTF("Example task creation failed!.\r\n");
		while (1)
			;
	}
	else {
		setvbuf (stdout, NULL, _IONBF, 0);
		PRINTF("Empezo\n");
	}
	vTaskStartScheduler();
	for (;;)
		;
    return 0;
}

static void example_task(void *pvParameters) {
	if(audio_player_init(4)==AUDIO_PLAYER_SUCCESS)
		audio_player_play_audio(AUDIO_PLAYER_BAD_SPO2);

//	vTaskDelay(pdMS_TO_TICKS(2000));
//	audio_player_stop_curr_audio();
//
//	vTaskDelay(pdMS_TO_TICKS(2000));
//	audio_player_play_audio(AUDIO_PLAYER_BAD_SPO2);

	for (;;) {
		vTaskSuspend(NULL);
	}

}

