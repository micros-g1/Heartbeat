/*
 * flash.c
 *
 *  Created on: 1 feb. 2021
 *      Author: Tomas
 */
//#include "fsl_debug_console.h"
#include <drv/flashmem.h>
#include "drivers/fsl_flash.h"
#include "drivers/fsl_uart.h"
#include "fsl_ftfx_adapter.h"

#define START_ADDRESS	0x3E800
#define LAST_ADDRESS	0xFFFF0

static void error_trap();

static flash_config_t config;
static uint32_t pflashSectorSize;

static flashmem_file_t file_array[FLASHMEM_FILE_COUNT];

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
//	else
//		PRINTF("FLASH_Init\r\n");
#endif

	result = FLASH_GetProperty(&config, kFLASH_PropertyPflash0BlockBaseAddr, &pflashBlockBase);
	if(result != kStatus_FTFx_Success)
		error_trap();
#ifdef FLASHMEM_PROGRAM
//	else
//		PRINTF("Get Base Address Property\r\n");
#endif
	result = FLASH_GetProperty(&config, kFLASH_PropertyPflash0TotalSize, &pflashTotalSize);
	if(result != kStatus_FTFx_Success)
		error_trap();
#ifdef FLASHMEM_PROGRAM
//	else
//		PRINTF("Get Flash Total Size\r\n");
#endif

	result = FLASH_GetProperty(&config, kFLASH_PropertyPflash0SectorSize, &pflashSectorSize);
	if(result != kStatus_FTFx_Success)
		error_trap();
#ifdef FLASHMEM_PROGRAM
//	else
//		PRINTF("Get Flash Sector Size\r\n");

	/* print welcome message */
//	PRINTF("\r\n Flash Example Start \r\n");
	/* Print flash information - PFlash. */
//	PRINTF("\r\n Flash Information: ");
//	PRINTF("\r\n Total Program Flash Size:\t%d KB, Hex: (0x%x)", (unsigned int)(pflashTotalSize / 1024),
//			(unsigned int)pflashTotalSize);
//	PRINTF("\r\n Program Flash Sector Size:\t%d KB, Hex: (0x%x) ", (unsigned int)(pflashSectorSize / 1024),
//			(unsigned int)pflashSectorSize);
#endif

    result = FLASH_GetSecurityState(&config, &sec_status);
    if (result != kStatus_FTFx_Success){
//		PRINTF("FLASH_Init error  result = %d \r\n", (int) result);
		error_trap();
    }

#ifdef FLASHMEM_PROGRAM
	/* Print security status. */
	switch (sec_status)
	{
		case kFTFx_SecurityStateNotSecure:
//			PRINTF("\r\n Flash is UNSECURE!");
			break;
		case kFTFx_SecurityStateBackdoorEnabled:
//			PRINTF("\r\n Flash is SECURE, BACKDOOR is ENABLED!");
			break;
		case kFTFx_SecurityStateBackdoorDisabled:
//			PRINTF("\r\n Flash is SECURE, BACKDOOR is DISABLED!");
			break;
		default:
			break;
	}
//	PRINTF("\r\n");

    /* Debug message for user. */
    /* Erase several sectors on upper pflash block where there is no code */
//    PRINTF("\r\n Erase a sector of flash");
#endif

    return result == kStatus_FTFx_Success ? FLASHMEM_SUCCESS : FLASHMEM_FAILURE;

}


#ifdef FLASHMEM_PROGRAM
flashmem_state_t flashmem_program(){

	// Borrar la flash
    uint32_t result;

    result = FLASH_Erase(&config, START_ADDRESS, LAST_ADDRESS - START_ADDRESS, kFTFx_ApiEraseKey);
    if (result != kStatus_FTFx_Success){
        error_trap();
    }

    /* Verify sector if it's been erased. */
    result = FLASH_VerifyErase(&config, START_ADDRESS, LAST_ADDRESS - START_ADDRESS, kFTFx_MarginValueUser);
    if (result != kStatus_FTFx_Success){
        error_trap();
    }

    /* Print message for user. */
//    PRINTF("\r\n Successfully Erased Sector 0x%x -> 0x%x\r\n", (unsigned int)START_ADDRESS,
//    		(unsigned int)(START_ADDRESS + (LAST_ADDRESS - START_ADDRESS)));


    // programar flash
	uint8_t n_sectors;
	uint8_t temp_buffer[FLASH0_FEATURE_PFLASH_BLOCK_SECTOR_SIZE];
	uint32_t flashed_sectors_counter = 0;
    uint32_t failAddr, failDat;
	uint8_t check = 0;

	UART_WriteBlocking(UART0, "Go\n", strlen("Go\n"));

	for(int j = 0; j < FLASHMEM_FILE_COUNT; j++){
		// esperar el numero de sectores que debo recibir
		UART_ReadBlocking(UART0, &n_sectors, 1);
		file_array[j].length = FLASH0_FEATURE_PFLASH_BLOCK_SECTOR_SIZE * n_sectors;
		file_array[j].start_address = START_ADDRESS + flashed_sectors_counter * FLASH0_FEATURE_PFLASH_BLOCK_SECTOR_SIZE;
		for(int i = 0; i < n_sectors; i++){
			UART_ReadBlocking(UART0, &temp_buffer, FLASH0_FEATURE_PFLASH_BLOCK_SECTOR_SIZE);
			UART_ReadBlocking(UART0, &check, sizeof(check));

			if (flashmem_checksum(temp_buffer, FLASH0_FEATURE_PFLASH_BLOCK_SECTOR_SIZE) != check){
				error_trap();
			}

			/* Print message for user. */
//			PRINTF("\r\n Program a buffer to a sector of flash ");

			uint32_t dest_address = START_ADDRESS + flashed_sectors_counter * FLASH0_FEATURE_PFLASH_BLOCK_SECTOR_SIZE;

			result = FLASH_Program(&config, dest_address, temp_buffer, sizeof(temp_buffer));
			if (kStatus_FLASH_Success != result){
				error_trap();
			}

			/* Program Check user margin levels */
			result = FLASH_VerifyProgram(&config, dest_address, sizeof(temp_buffer), temp_buffer, kFTFx_MarginValueUser, &failAddr,
										 &failDat);
			if (result != kStatus_FTFx_Success){
				error_trap();
			}
//			PRINTF("\r\n Successfully Programmed and Verified Location 0x%x -> 0x%x \r\n", (unsigned int)dest_address,
//			           (unsigned int)(dest_address + sizeof(temp_buffer)));

			UART_WriteBlocking(UART0, "OK\n", strlen("OK\n"));

			flashed_sectors_counter++;
//			PRINTF("\r\nChunk Numero %d", flashed_sectors_counter);
		}
	}

    return result == kStatus_FTFx_Success ? FLASHMEM_SUCCESS : FLASHMEM_FAILURE;
}
#endif

flashmem_file_t flashmem_get_file(flashmem_file_id_t file_id){
	// Check if file id is in range
	flashmem_file_t ret;
	if(file_id < FLASHMEM_FILE_COUNT)
		ret =  file_array[file_id];
	else{
		ret.start_address = 0x00;
		ret.length = 0U;
	}
	return ret;

}

/*
If you pass it a block of data without a checksum on the end, it will give you the checksum.
If you pass it a block with the checksum on the end, it will give you zero for a good checksum, or non-zero if the checksum is bad.
*/

uint8_t flashmem_checksum (uint8_t *ptr, uint32_t sz) {
    unsigned char chk = 0;
    while (sz-- != 0)
        chk -= *ptr++;
    return chk;
}
/*
* @brief Gets called when an error occurs.
*
* @details Print error message and trap forever.
*/
static void error_trap(){
//    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1);
}
