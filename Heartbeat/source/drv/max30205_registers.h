/*
 * max30205_registers.h
 *
 *  Created on: 13 ene. 2021
 *      Author: Tomas
 */

#ifndef DRV_MAX30205_REGISTERS_H_
#define DRV_MAX30205_REGISTERS_H_

#include <stdint.h>

// Information from 'MAX30205 Human Body Temperature Sensor' Rev. Oct 24, 2016
// https://datasheets.maximintegrated.com/en/ds/MAX30205.pdf

//WRITE ADDRESS: 0x90 = 1001 0000, READ ADDRESS: 1001 0001 (A0 & A1 & A2 to GND)
//ADDR: 1010111x: 0100 1000 = 0x48
#define MAX30205_I2C_ADDRESS 0x48

#define MAX30205_MASK_GENERATE(BITSTART,LENGTH)  ( ( (1 << (LENGTH)) - 1 ) << (  (BITSTART) + 1 - (LENGTH)) )
#define MAX30205_APPLY_BITSTART_LENGTH(DATA,BITSTART,LENGTH)  ( ( (DATA) << (  (BITSTART) + 1 - (LENGTH)) ) & MAX30102_MASK_GENERATE(BITSTART,LENGTH) )
#define MAX30205_RECOVER_BITSTART_LENGTH(DATA,BITSTART,LENGTH) ( ( (DATA) & MAX30205_MASK_GENERATE(BITSTART,LENGTH) ) >> (  (BITSTART) + 1 - (LENGTH)) )

#define MAX30205_ONE_SHOT_BIT_START 7
#define MAX30205_ONE_SHOT_BIT_LENGTH 1
#define MAX30205_NOT_TIMEOUT_BIT_START 6
#define MAX30205_NOT_TIMEOUT_BIT_LENGTH 1
#define MAX30205_DATA_FORMAT_BIT_START 5
#define MAX30205_DATA_FORMAT_BIT_LENGTH 1
#define MAX30205_FAULT_QUEUE_BIT_START 4
#define MAX30205_FAULT_QUEUE_BIT_LENGTH 2
#define MAX30205_OS_POLARITY_BIT_START 2
#define MAX30205_OS_POLARITY_BIT_LENGTH 1
#define MAX30205_NOT_COMP_INT_BIT_START 1
#define MAX30205_NOT_COMP_INT_BIT_LENGTH 1
#define MAX30205_SHUTDOWN_BIT_START 0
#define MAX30205_SHUTDOWN_BIT_LENGTH 1

typedef union
{
	struct{
		unsigned int shutdown : 1;
		unsigned int not_comparator_interrupt : 1;
		unsigned int os_polarity : 1;
		unsigned int fault_queue : 2;
		unsigned int data_format : 1;
		unsigned int not_timeout : 1;
		unsigned int one_shot : 1;
	};
	uint8_t val;
}__attribute__((__packed__, aligned(1))) max30205_config_t;

typedef enum
{
	MAX30205_FAULT_QUEUE_0 = 0b00,
	MAX30205_FAULT_QUEUE_2 = 0b01,
	MAX30205_FAULT_QUEUE_4 = 0b10,
	MAX30205_FAULT_QUEUE_6 = 0b11
}max30205_fault_queue_t;

typedef enum
{
	MAX30205_TEMP_ADDR = 0x00,
	MAX30205_CONFIG_ADDR = 0x01,
	MAX30205_THYS_ADDR = 0x02,
	MAX30205_TOS_ADDR = 0x03
}max30205_addr_t;


#endif /* DRV_MAX30205_REGISTERS_H_ */
