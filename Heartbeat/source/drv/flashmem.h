/*
 * flash.h
 *
 *  Created on: 1 feb. 2021
 *      Author: Tomas
 */

#ifndef DRV_FLASHMEM_H_
#define DRV_FLASHMEM_H_

#include <stdbool.h>
#include <stdint.h>

#define FLASHMEM_PROGRAM
//#define FLASHMEM_TEST_FLASH

#define FLASHMEM_BUFFER_LEN 4
#define FLASHMEM_DEST_ADDR	0x3e800
#define FLASHMEM_FILE_COUNT	1

typedef enum
{
	FLASHMEM_SUCCESS = true,
	FLASHMEM_FAILURE = false
}flashmem_state_t;

typedef struct{
	uint32_t start_address;
	uint32_t length;
}flashmem_file_t;

typedef enum{
	FLASHMEM_FILE_1 = 0x00,
	FLASHMEM_FILE_2 = 0x01,
	FLASHMEM_FILE_3 = 0x02
}flashmem_file_id_t;

flashmem_state_t flashmem_init();
flashmem_file_t flashmem_get_file(flashmem_file_id_t file_id);
uint8_t flashmem_checksum (uint8_t *ptr, uint32_t sz);

#ifdef FLASHMEM_PROGRAM
flashmem_state_t flashmem_program();
#endif

#endif /* DRV_FLASHMEM_H_ */
