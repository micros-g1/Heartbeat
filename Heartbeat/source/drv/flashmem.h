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

//#define FLASHMEM_PROGRAM
#define FLASHMEM_BUFFER_LEN 	4
#define FLASHMEM_DEST_ADDR	0xFF000

typedef enum
{
	FLASHMEM_SUCCESS = true,
	FLASHMEM_FAILURE = false
}flashmem_state_t;

flashmem_state_t flashmem_init();

#ifdef FLASHMEM_PROGRAM
flashmem_state_t flashmem_program();
#endif

#endif /* DRV_FLASHMEM_H_ */
