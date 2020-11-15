/*
 * max30102.c
 *
 *  Created on: 14 Nov 2020
 *      Author: grein
 */

#include "max30102.h"
#include "max30102_registers.h"
#include <limits.h>
#include <stdbool.h>
#include <peripherals.h>
#include "i2c.h"

#define MAX30100_TFRAC_STEP_C	1.0/16
#define MAX30100_TINT_SETP_C	1
#define MAX30100_CALCULATE_TEMP_C(tint,tfrac) ((float)(MAX30100_TINT_SETP_C*((int8_t)(tint)) + MAX30100_TFRAC_STEP_C*((uint8_t)(tfrac))))


i2c_handle_t* i2c = NULL;

max30102_state_t max30102_init()
{
	i2c = &I2CA_rtosHandle;
	i2c_init(i2c);
	uint8_t part_id;
	max30102_state_t state = max30102_get_part_id(&part_id);
	state = state == MAX30102_SUCCESS? (part_id == MAX30102_PART_ID? MAX30102_SUCCESS : MAX30102_FAILURE) : state;
	state = state == MAX30102_SUCCESS? max30102_reset() : state;
	state = state == MAX30102_SUCCESS? max30102_wait_reset_ready() : state;
	return state;
}

max30102_state_t max30102_trigger_temp_read()
{
	if (!i2c) return MAX30102_FAILURE;
	uint8_t data = MAX30102_MASK_GENERATE(MAX30102_DIE_TEMP_EN_BIT_START,MAX30102_DIE_TEMP_EN_BIT_LENGTH);
	uint8_t mask = data;
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_DIE_TEMPERATURE_CONFIG_ADDR, mask, data)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_is_temp_read_ready(bool* rdy)
{
	if (!i2c) return MAX30102_FAILURE;
	uint8_t data;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_DIE_TEMP_EN_BIT_START,MAX30102_DIE_TEMP_EN_BIT_LENGTH);
	max30102_state_t state = i2c_read_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_DIE_TEMPERATURE_CONFIG_ADDR, mask, &data)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
	if(state == MAX30102_SUCCESS)
		*rdy = !data;
	return state;
}

max30102_state_t max30102_wait_temp_read_ready()
{
	if(!i2c) return MAX30102_FAILURE;
	bool ready = false;
	max30102_state_t state;
	do
	{
		state = max30102_is_temp_read_ready(&ready);
	}while(!ready && state == MAX30102_SUCCESS);
	return state;
}

max30102_state_t max30102_get_temperature_c(float* temp)
{
	if (!i2c) return MAX30102_FAILURE;
	uint8_t t_data[2];
	max30102_state_t state = i2c_read_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_DIE_TEMPERATURE_INTEGER_ADDR, 2, t_data)?
						MAX30102_SUCCESS : MAX30102_FAILURE;
	if(state == MAX30102_SUCCESS)
	{
		uint8_t integer = MAX30102_RECOVER_BITSTART_LENGTH(t_data[0],MAX30102_DIE_TEMP_INTEGER_BIT_START,MAX30102_DIE_TEMP_INTEGER_LENGTH);
		uint8_t frac = MAX30102_RECOVER_BITSTART_LENGTH(t_data[1],MAX30102_DIE_TEMP_FRACTION_BIT_START,MAX30102_DIE_TEMP_FRACTION_LENGTH);
		*temp = MAX30100_CALCULATE_TEMP_C(integer,frac);
	}
	return state;
}

max30102_state_t max30102_reset()
{
	if(!i2c) return MAX30102_FAILURE;
	max30102_mode_configuration_t mode_config;
	memset(&mode_config, 0, sizeof(mode_config));
	max30102_mode_configuration_t mode_config_mask = mode_config;
	mode_config.reset = true;
	mode_config_mask.reset = true;
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, mode_config_mask.val, (uint8_t) mode_config.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_is_reset_ready(bool* rdy)
{
	if(!i2c) return MAX30102_FAILURE;
	max30102_mode_configuration_t mode_config;
	max30102_state_t state =  i2c_read_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, (uint8_t*) &mode_config.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
	if(state == MAX30102_SUCCESS)
		*rdy = !mode_config.reset;
	return state;
}

max30102_state_t max30102_wait_reset_ready()
{
	if(!i2c) return MAX30102_FAILURE;
	bool ready = false;
	max30102_state_t state;
	do
	{
		state = max30102_is_reset_ready(&ready);
	}while(!ready && state == MAX30102_SUCCESS);
	return state;
}

max30102_state_t max30102_get_revision_id(uint8_t* rev_id)
{
	if(!i2c) return MAX30102_FAILURE;
	max30102_state_t state = i2c_read_byte_addr8(i2c, MAX30102_I2C_ADDRESS, (uint8_t) MAX30102_REVISION_ID_ADDR, rev_id)?
			MAX30102_SUCCESS : MAX30102_FAILURE;
	if(state == MAX30102_SUCCESS)
			*rev_id = MAX30102_RECOVER_BITSTART_LENGTH(*rev_id,MAX30102_REVISION_ID_BIT_START,MAX30102_REVISION_ID_BIT_LENGTH);
	return state;
}

max30102_state_t max30102_get_part_id(uint8_t* part_id)
{
	if(!i2c) return MAX30102_FAILURE;
	max30102_state_t state = i2c_read_byte_addr8(i2c, MAX30102_I2C_ADDRESS, (uint8_t) MAX30102_PART_ID_ADDR, part_id)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
	if(state == MAX30102_SUCCESS)
		*part_id = MAX30102_RECOVER_BITSTART_LENGTH(*part_id,MAX30102_PART_ID_BIT_START,MAX30102_PART_ID_BIT_LENGTH);
	return state;
}

