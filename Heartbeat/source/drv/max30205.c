/*
 * max30205.c
 *
 *  Created on: 13 ene. 2021
 *      Author: taomasgonzalez
 */

#include "max30205.h"
#include "i2c.h"
#define MAX30205_TEMP_N_BYTES 2

static i2c_handle_t* i2c = NULL;
max30205_state_t max30205_init(){
//	i2c = &I2CA_rtosHandle;
	i2c_init(i2c);
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
