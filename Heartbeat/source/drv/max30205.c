/*
 * max30205.c
 *
 *  Created on: 13 ene. 2021
 *      Author: taomasgonzalez
 */

#include "drv/max30205.h"
#include "drv/i2c.h"
#include "drv/max30205_registers.h"
#include <peripherals.h>

#define MAX30205_TEMP_N_BYTES 2
#define MAX30205_TOS_N_BYTES 2
#define MAX30205_THYST_N_BYTES 2

static i2c_handle_t* i2c = NULL;
max30205_state_t max30205_init(){

	//Power-On Reset Value
	/*
	 * These default POR values correspond to the following modes of operation:
	 	 * Comparator mode
	 	 * OS active low
	 	 * 1 fault, fault queue
		 * Normal data format
		 * Timeout enabled for MAX30205
	 */

	i2c = &I2CA_rtosHandle;
	i2c_init(i2c);
	return MAX30205_SUCCESS;
}

max30205_state_t max30205_temp_read(float *temp){

	if (!i2c) return MAX30205_FAILURE;

	uint8_t read_data[MAX30205_TEMP_N_BYTES];

	max30205_state_t read_success = i2c_read_addr8(i2c, MAX30205_I2C_ADDRESS, MAX30205_TEMP_ADDR, MAX30205_TEMP_N_BYTES, read_data)?
										MAX30205_SUCCESS : MAX30205_FAILURE;

	if(read_success == MAX30205_SUCCESS){
		*temp = (uint16_t) read_data[0] << 8 | read_data[1];
		*temp = *temp * 0.00390625;
	}

	return read_success;
}


max30205_state_t max30205_set_config(max30205_config_t *config){
	if(!i2c) return MAX30205_FAILURE;

	return i2c_write_byte_addr8(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, config->val)?
					MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_set_one_shot(bool one_shot){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.one_shot = one_shot;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_ONE_SHOT_BIT_START, MAX30205_ONE_SHOT_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_set_not_timeout(bool not_timeout){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.not_timeout = not_timeout;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_NOT_TIMEOUT_BIT_START, MAX30205_NOT_TIMEOUT_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_set_data_format(bool data_format){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.data_format = data_format;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_DATA_FORMAT_BIT_START, MAX30205_DATA_FORMAT_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_set_fault_queue(max30205_fault_queue_t fault_queue){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.fault_queue = fault_queue;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_FAULT_QUEUE_BIT_START, MAX30205_FAULT_QUEUE_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}

max30205_state_t max30205_set_os_polarity(bool os_polarity){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.one_shot = os_polarity;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_OS_POLARITY_BIT_START, MAX30205_OS_POLARITY_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_not_comparator_interrupt(bool not_comparator_interrupt){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.not_comparator_interrupt = not_comparator_interrupt;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_NOT_COMP_INT_BIT_START, MAX30205_NOT_COMP_INT_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_shutdown(bool shutdown){
	if(!i2c) return MAX30205_FAILURE;

	max30205_config_t config;
	memset(&config, 0, sizeof(config));
	config.shutdown = shutdown;

	uint8_t mask = MAX30205_MASK_GENERATE(MAX30205_SHUTDOWN_BIT_START, MAX30205_SHUTDOWN_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30205_I2C_ADDRESS, MAX30205_CONFIG_ADDR, mask, config.val)?
				MAX30205_SUCCESS : MAX30205_FAILURE;
}


max30205_state_t max30205_set_thyst(float thyst){
	uint8_t data[MAX30205_TOS_N_BYTES];

	if(thyst < 0){
		data[0] = (uint8_t)(-thyst);
		data[0] &= 0x7F;
		data[1] = - thyst + (int16_t)(thyst);
	}
	else{
		data[0] = thyst;
		data[1] = thyst - (int16_t)(thyst);
	}

	return i2c_write_addr8(i2c, MAX30205_I2C_ADDRESS, MAX30205_THYS_ADDR, MAX30205_TOS_N_BYTES, data) ?
			MAX30205_SUCCESS : MAX30205_FAILURE;
}
max30205_state_t max30205_get_thyst(float *thyst){
	uint8_t data[MAX30205_THYST_N_BYTES];

	max30205_state_t success = i2c_read_addr8(i2c, MAX30205_I2C_ADDRESS, MAX30205_THYS_ADDR,
			MAX30205_TOS_N_BYTES, data) ? MAX30205_SUCCESS: MAX30205_FAILURE;

	if(success == MAX30205_SUCCESS){
		*thyst = (uint16_t) (data[0] & 0x07)  << 8 | data[1];
		if(data[0] & 0x80)
			*thyst = *thyst * -0.00390625;
		else
			*thyst = *thyst * 0.00390625;
	}
	return success;
}

max30205_state_t max30205_set_tos(float tos){
	uint8_t data[MAX30205_TOS_N_BYTES];

	if(tos < 0){
		data[0] = (uint8_t)(-tos);
		data[0] &= 0x7F;
		data[1] = - tos + (int16_t)(tos);
	}
	else{
		data[0] = tos;
		data[1] = tos - (int16_t)(tos);
	}

	return i2c_write_addr8(i2c, MAX30205_I2C_ADDRESS, MAX30205_TOS_ADDR, MAX30205_TOS_N_BYTES, data) ?
			MAX30205_SUCCESS : MAX30205_FAILURE;

}
max30205_state_t max30205_get_tos(float *tos){
	uint8_t data[MAX30205_TOS_N_BYTES];
	max30205_state_t success = i2c_read_addr8(i2c, MAX30205_I2C_ADDRESS, MAX30205_TOS_ADDR,
			MAX30205_TOS_N_BYTES, data) ? MAX30205_SUCCESS: MAX30205_FAILURE;

	if(success == MAX30205_SUCCESS){
		*tos = (uint16_t) (data[0] & 0x7F)  << 8 | data[1];
		if(data[0] & 0x80)
			*tos = *tos * -0.00390625;
		else
			*tos = *tos * 0.00390625;
	}
	return success;
}
