/*
 * max30102.c
 *
 *  Created on: 14 Nov 2020
 *      Authors: grein, taomasgonzalez
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
static max30102_state_t set_mode_configuration(max30102_mode_configuration_t *conf);
//interrupts to be enabled should be set.
static max30102_state_t max30102_enable_interrupts(max30102_interrupt_enable_t *enables, max30102_addr_t interrupt_enable_addr);
// interrupts to be disabled should be cleared, the rest of the bits should be set.
static max30102_state_t max30102_disable_interrupts(max30102_interrupt_enable_t *enables, max30102_addr_t interrupt_enable_addr);

static max30102_state_t max30102_hard_reset();
static max30102_state_t max30102_is_reset_ready(bool* reset_ready);
static max30102_state_t max30102_wait_reset_ready();

max30102_state_t max30102_init(max30102_mode_t initial_mode)
{
	i2c = &I2CA_rtosHandle;
	i2c_init(i2c);
	uint8_t part_id;
	max30102_state_t state = max30102_get_part_id(&part_id);
	state = state == MAX30102_SUCCESS? (part_id == MAX30102_PART_ID? MAX30102_SUCCESS : MAX30102_FAILURE) : state;
	state = state == MAX30102_SUCCESS? max30102_hard_reset() : state;
	state = state == MAX30102_SUCCESS? max30102_wait_reset_ready() : state;

	max30102_mode_configuration_t conf;
	memset(&conf, 0, sizeof(conf));
	conf.mode = initial_mode;
	state = state == MAX30102_SUCCESS? set_mode_configuration(&conf) : state;

	max30102_interrupt_enable_t enables;
	memset(&enables, 0, sizeof(enables));
	enables.die_temp_rdy_en = true;
	state = state == MAX30102_SUCCESS? max30102_enable_interrupts(&enables, MAX30102_INTERRUPT_ENABLE_2_ADDR): state;

	return state;
}


max30102_state_t max30102_trigger_temp_read()
{
	if (!i2c) return MAX30102_FAILURE;
	uint8_t data = MAX30102_MASK_GENERATE(MAX30102_DIE_TEMP_EN_BIT_START, MAX30102_DIE_TEMP_EN_BIT_LENGTH);
	uint8_t mask = data;
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_DIE_TEMPERATURE_CONFIG_ADDR, mask, data)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_is_temp_read_ready(bool* rdy)
{
	if (!i2c) return MAX30102_FAILURE;
	uint8_t data;
	max30102_state_t state = i2c_read_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_STATUS_2_ADDR, 0x02, &data)?
						MAX30102_SUCCESS : MAX30102_FAILURE;

	*rdy = (state == MAX30102_SUCCESS) && data;

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

max30102_state_t max30102_hard_reset()
{
	if(!i2c) return MAX30102_FAILURE;
	max30102_mode_configuration_t mode_config;
	memset(&mode_config, 0, sizeof(mode_config));
	max30102_mode_configuration_t mode_config_mask = mode_config;
	mode_config.reset = true;
	mode_config_mask.reset = true;
	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, mode_config_mask.val)?
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

max30102_state_t set_mode_configuration(max30102_mode_configuration_t *conf){
	if(!i2c) return MAX30102_FAILURE;

	max30102_mode_configuration_t mode_config_mask;
	mode_config_mask.val = conf->val;
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, mode_config_mask.val, (uint8_t) conf->val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}


max30102_state_t max30102_enable_interrupts(max30102_interrupt_enable_t *enables, max30102_addr_t interrupt_enable_addr){
	if(!i2c) return MAX30102_FAILURE;

	uint8_t *part = (interrupt_enable_addr == MAX30102_INTERRUPT_ENABLE_2_ADDR) ? enables->byte : enables->byte + 1;
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, interrupt_enable_addr, *part, *part)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_disable_interrupts(max30102_interrupt_enable_t *enables, max30102_addr_t interrupt_enable_addr){
	if(!i2c) return MAX30102_FAILURE;

	uint8_t *part = (interrupt_enable_addr == MAX30102_INTERRUPT_ENABLE_1_ADDR) ? enables->byte : enables->byte + 1;
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, interrupt_enable_addr, ~(*part), *part)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_fifo_config(max30102_fifo_configuration_t *config){
	if(!i2c) return MAX30102_FAILURE;

	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_CONFIGURATION_ADDR, config->val)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}
max30102_state_t max30102_set_spo2_config(max30102_spo2_configuration_t *config){
	if(!i2c) return MAX30102_FAILURE;
	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_SPO2_CONFIGURATION_ADDR, config->val)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_led_current(uint8_t curr_lvl, max30102_addr_t led_addr){
	if(!i2c) return MAX30102_FAILURE;
	if(led_addr != MAX30102_LED_PULSE_AMPLITUDE_ADDR_1 && led_addr != MAX30102_LED_PULSE_AMPLITUDE_ADDR_2) return MAX30102_FAILURE;
	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, led_addr, curr_lvl)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

