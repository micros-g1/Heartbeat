/*
 * flash.c
 *
 *  Created on: 1 feb. 2021
 *      Author: Tomas
 */
#include "fsl_debug_console.h"
#include <drv/flashmem.h>
#include "drivers/fsl_flash.h"


static void error_trap();

static flash_config_t config;
static uint32_t dest_adrss; /* Address of the target location */
static uint32_t pflashSectorSize;


flashmem_state_t flashmem_init(){
	ftfx_security_state_t sec_status = kFTFx_SecurityStateNotSecure;

    uint32_t pflashBlockBase = 0;
    uint32_t pflashTotalSize = 0;
    pflashSectorSize = 0;


	memset(&config, 0, sizeof(flash_config_t));

	status_t result = FLASH_Init(&config);

	if(result != kStatus_FTFx_Success)
		error_trap();

#ifdef FLASHMEM_PROGRAM
	else
		PRINTF("FLASH_Init\r\n");
#endif

	result = FLASH_GetProperty(&config, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
	if(result != kStatus_FTFx_Success)
		error_trap();
#ifdef FLASHMEM_PROGRAM
	else
		PRINTF("Get Base Address Property\r\n");
#endif
	result = FLASH_GetProperty(&config, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
	if(result != kStatus_FTFx_Success)
		error_trap();
#ifdef FLASHMEM_PROGRAM
	else
		PRINTF("Get Flash Total Size\r\n");
#endif

	result = FLASH_GetProperty(&config, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);
	if(result != kStatus_FTFx_Success)
		error_trap();
#ifdef FLASHMEM_PROGRAM
	else
		PRINTF("Get Flash Sector Size\r\n");

	/* print welcome message */
	PRINTF("\r\n Flash Example Start \r\n");
	/* Print flash information - PFlash. */
	PRINTF("\r\n Flash Information: ");
	PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (pflashTotalSize / 1024), pflashTotalSize);
	PRINTF("\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (pflashSectorSize / 1024), pflashSectorSize);
#endif

    result = FLASH_GetSecurityState(&config, &sec_status);
    if (result != kStatus_FTFx_Success){
		PRINTF("FLASH_Init error  result = %d \r\n", result);
		error_trap();
    }

#ifdef FLASHMEM_PROGRAM
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
#endif
    /* Erase a sector from destAdrss. */
    dest_adrss = pflashBlockBase + (pflashTotalSize - pflashSectorSize);

    return result == kStatus_FTFx_Success ? FLASHMEM_SUCCESS : FLASHMEM_FAILURE;

}


#ifdef FLASHMEM_PROGRAM
flashmem_state_t flashmem_program(){
    uint32_t i, failAddr, failDat;

    uint32_t buffer[FLASHMEM_BUFFER_LEN] = {0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC, 0xDDDDDDDD}; /* Buffer for program */
    uint32_t result;

    result = FLASH_Erase(&config, dest_adrss, pflashSectorSize, kFTFx_ApiEraseKey);
    if (result != kStatus_FTFx_Success){
        error_trap();
    }

    /* Verify sector if it's been erased. */
    result = FLASH_VerifyErase(&config, dest_adrss, pflashSectorSize, kFTFx_MarginValueUser);
    if (result != kStatus_FTFx_Success){
        error_trap();
    }

    /* Print message for user. */
    PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", dest_adrss, (dest_adrss + pflashSectorSize));

    /* Print message for user. */
    PRINTF("\r\n Program a buffer to a sector of flash ");

    result = FLASH_Program(&config, dest_adrss, (uint8_t *)buffer, FLASHMEM_BUFFER_LEN * sizeof(buffer[0]));
    if (kStatus_FLASH_Success != result){
        error_trap();
    }

    /* Program Check user margin levels */
    result = FLASH_VerifyProgram(&config, dest_adrss, FLASHMEM_BUFFER_LEN * sizeof(buffer[0]), (uint8_t *)buffer, kFTFx_MarginValueUser, &failAddr,
                                 &failDat);
    if (result != kStatus_FTFx_Success){
        error_trap();
    }
    PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", dest_adrss,
           (dest_adrss + FLASHMEM_BUFFER_LEN * sizeof(buffer[0])));

    /* Print finished message. */
    PRINTF("\r\n End of Flash Example \r\n");

    return result == kStatus_FTFx_Success ? FLASHMEM_SUCCESS : FLASHMEM_FAILURE;
}
#endif
/*
* @brief Gets called when an error occurs.
*
* @details Print error message and trap forever.
*/
void error_trap(){
    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1);
}
