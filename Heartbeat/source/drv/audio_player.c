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

//https://community.nxp.com/t5/Kinetis-Microcontrollers/
// 			How-to-write-my-own-data-to-the-flash-of-K64-above-the-program/td-p/653690

#include "drv/audio_player.h"
#include "drivers/fsl_flash.h"
#include "fsl_debug_console.h"

#define BUFFER_LEN 4

static void error_trap();
static void flash_init();

/*
 * Make sure that your code is not (by mistake) re-writing to the Flash
 * the next time that it runs (without first erasing it) since a second write to a phrase,
 * without first erasing the sector that is in, is an invalid operation that can damage the flash
 * and cause it to behave strangely (often causing hard-faults to occur when the corrupted area is read,
 * where debuggers will tend to display the content as "-- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- ").
 */
audio_player_state_t audio_player_init(){
	flash_init();
	return AUDIO_PLAYER_SUCCESS;
}


void flash_init(){
	flash_config_t config;

	ftfx_security_state_t sec_status = kFTFx_SecurityStateNotSecure;
    uint32_t dest_adrss; /* Address of the target location */
    uint32_t i, failAddr, failDat;
    uint32_t buffer[BUFFER_LEN]; /* Buffer for program */

    uint32_t pflashBlockBase = 0;
    uint32_t pflashTotalSize = 0;
    uint32_t pflashSectorSize = 0;

	memset(&config, 0, sizeof(flash_config_t));

	status_t result = FLASH_Init(&config);

	if(result != kStatus_Success)
		error_trap();
	else
		PRINTF("FLASH_Init\r\n");

	FLASH_GetProperty(&config, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
	FLASH_GetProperty(&config, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
	FLASH_GetProperty(&config, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);

	/* print welcome message */
	PRINTF("\r\n Flash Example Start \r\n");
	/* Print flash information - PFlash. */
	PRINTF("\r\n Flash Information: ");
	PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
	PRINTF("\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (pflashSectorSize / 1024), pflashSectorSize);

    result = FLASH_GetSecurityState(&config, &sec_status);
    if (result != kStatus_Success){
		PRINTF("FLASH_Init error  result = %d \r\n", result);
		error_trap();
    }

	/* Print security status. */
	switch (sec_status)
	{
		case kFTFx_SecurityStateNotSecure:
			PRINTF("\r\n Flash is UNSECURE!");
			break;
		case kFTFx_SecurityStateBackdoorEnabled:
			PRINTF("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
			break;
		case kFTFx_SecurityStateBackdoorDisabled:
			PRINTF("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
			break;
		default:
			break;
	}
	PRINTF("\r\n");

    /* Debug message for user. */
    /* Erase several sectors on upper pflash block where there is no code */
    PRINTF("\r\n Erase a sector of flash");

    /* Erase a sector from destAdrss. */
    dest_adrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
    result = FLASH_Erase(&config, dest_adrss, pflashSectorSize, kFTFx_ApiEraseKey);
    if (result != kStatus_Success){
        error_trap();
    }

    /* Verify sector if it's been erased. */
    result = FLASH_VerifyErase(&config, dest_adrss, pflashSectorSize, kFTFx_MarginValueUser);
    if (result != kStatus_Success){
        error_trap();
    }

    /* Print message for user. */
    PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", dest_adrss, (dest_adrss + pflashSectorSize));

    /* Print message for user. */
    PRINTF("\r\n Program a buffer to a sector of flash ");

    /* Prepare buffer. */
    for (i = 0; i < BUFFER_LEN; i++){
        buffer[i] = i;
    }
    dest_adrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);
    result = FLASH_Program(&config, dest_adrss, buffer, sizeof(buffer));
    if (kStatus_FLASH_Success != result){
        error_trap();
    }

    /* Program Check user margin levels */
    result = FLASH_VerifyProgram(&config, dest_adrss, sizeof(buffer), buffer, kFTFx_MarginValueUser, &failAddr,
                                 &failDat);
    if (result != kStatus_Success){
        error_trap();
    }
    PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", dest_adrss,
           (dest_adrss + sizeof(buffer)));

    /* Print finished message. */
    PRINTF("\r\n End of Flash Example \r\n");
    while (1)
    {
    }
}
/*
* @brief Gets called when an error occurs.
*
* @details Print error message and trap forever.
*/
void error_trap(){
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {
    }
}