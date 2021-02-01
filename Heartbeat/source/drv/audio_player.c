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
#include "drv/audio_player.h"
#include "fsl_debug_console.h"
#include "drivers/fsl_uart.h"

/*
 * Make sure that your code is not (by mistake) re-writing to the Flash
 * the next time that it runs (without first erasing it) since a second write to a phrase,
 * without first erasing the sector that is in, is an invalid operation that can damage the flash
 * and cause it to behave strangely (often causing hard-faults to occur when the corrupted area is read,
 * where debuggers will tend to display the content as "-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ").
 */
audio_player_state_t audio_player_init(){
	flashmem_init();

#ifdef FLASHMEM_PROGRAM
	flashmem_program();
#endif

	uint32_t buff_ram[FLASHMEM_BUFFER_LEN];
	memcpy(buff_ram, (uint32_t *)FLASHMEM_DEST_ADDR, sizeof(buff_ram));

	for(int i = 0; i < FLASHMEM_BUFFER_LEN; i++){
		PRINTF("\r\n%x", (unsigned int) buff_ram[i]);
	}

	return AUDIO_PLAYER_SUCCESS;
}
