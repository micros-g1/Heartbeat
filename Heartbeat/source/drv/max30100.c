/*
 * max30100_registers.c
 *
 *  Created on: 29 Oct 2020
 *      Author: grein
 */

#include "max30100.h"
#include "max30100_registers.h"
#include <limits.h>
#include <stdbool.h>
#include <peripherals.h>
#include "i2cdev.h"

#define MAX30100_BITS_PER_REGISTER 8
#define MAX30100_TFRAC_STEP_C	1.0/16
#define MAX30100_TINT_SETP_C	1
#define MAX30100_FORM_TEMP_C(tint,tfrac) ((float)(MAX30100_TINT_SETP_C*((int8_t)(tint)) + MAX30100_TFRAC_STEP_C*((uint8_t)(tfrac))))
#define MAX30100_FIFO_DEPTH 16

typedef i2c_handle_t max30100_i2c_handler_t;

static max30100_i2c_handler_t* i2c = NULL;

uint32_t current_sample;


max30100_state_t max30100_register_value_read(max30100_i2c_handler_t* i2c, uint8_t* valuep, uint8_t reg_addr, uint8_t field_bit_offset, uint8_t field_bit_length)
{
	if (!i2c) return MAX30100_FAILURE;
	bool result = false;
	if(field_bit_offset == 0 && field_bit_length == MAX30100_BITS_PER_REGISTER)
		result = i2cdevReadByte(i2c, MAX30100_I2C_ADDRESS, reg_addr, valuep);
	else
		result = i2cdevReadBits(i2c, MAX30100_I2C_ADDRESS, reg_addr, field_bit_offset+field_bit_length-1, field_bit_length, valuep);
	return result? MAX30100_SUCCESS : MAX30100_FAILURE;
}

max30100_state_t max30100_register_value_write(max30100_i2c_handler_t* i2c, uint8_t value, uint8_t reg_addr, uint8_t field_bit_offset, uint8_t field_bit_length)
{
	if (!i2c) return MAX30100_FAILURE;
	bool result = false;
		if(field_bit_offset == 0 && field_bit_length == MAX30100_BITS_PER_REGISTER)
			result = i2cdevWriteByte(i2c, MAX30100_I2C_ADDRESS, reg_addr, value);
		else
			result = i2cdevWriteBits(i2c, MAX30100_I2C_ADDRESS, reg_addr, field_bit_offset+field_bit_length-1, field_bit_length, value);
		return result? MAX30100_SUCCESS : MAX30100_FAILURE;
}

max30100_state_t max30100_init()
{
	i2c = &I2CA_rtosHandle;
	max30100_state_t state = MAX30100_SUCCESS;
	uint8_t part_id;
	state = state == MAX30100_SUCCESS? max30100_get_part_id(&part_id) : state;
	state = state == MAX30100_SUCCESS? (part_id == MAX30100_PART_ID? MAX30100_SUCCESS : MAX30100_FAILURE) : state;
	state = state == MAX30100_SUCCESS? max30100_reset() : state;
	state = state == MAX30100_SUCCESS? max30100_wait_reset_ready() : state;
	state = state == MAX30100_SUCCESS? max30100_clear_fifo() : state;
	return MAX30100_FAILURE;
}

max30100_state_t max30100_get_interrupt_flags(uint8_t* flags)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_STATUS_ADDR, 0, MAX30100_BITS_PER_REGISTER);
	*flags = val & MAX30100_ALL_INTERRUPTS_FLAGS_MASK;
	return ret_state;
}

max30100_state_t max30100_get_almost_full_flag(bool* flag)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_STATUS_ADDR, MAX30100_FIFO_ALMOST_FULL_FLAG_BIT_OFFSET, MAX30100_FIFO_ALMOST_FULL_FLAG_BIT_LENGTH);
	*flag = !!val;
	return ret_state;
}

max30100_state_t max30100_get_temp_ready_flag(bool* flag)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_STATUS_ADDR, MAX30100_TEMPERATURE_READY_FLAG_BIT_OFFSET, MAX30100_TEMPERATURE_READY_FLAG_BIT_LENGTH);
	*flag = !!val;
	return ret_state;
}

max30100_state_t max30100_get_hr_ready_flag(bool* flag)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_STATUS_ADDR, MAX30100_HEARTRATE_DATA_READY_BIT_OFFSET, MAX30100_HEARTRATE_DATA_READY_BIT_LENGTH);
	*flag = !!val;
	return ret_state;
}

max30100_state_t max30100_is_power_ready(bool* flag)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_STATUS_ADDR, MAX30100_POWER_READY_BIT_OFFSET, MAX30100_POWER_READY_BIT_LENGTH);
	*flag = !!val;
	return ret_state;
}

max30100_state_t max30100_fifo_almost_full_interrupt(bool enable)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, enable?1:0, MAX30100_INTERRUPT_ENABLE_ADDR, MAX30100_ENB_A_BIT_OFFSET, MAX30100_ENB_A_BIT_LENGTH);
}

max30100_state_t max30100_fifo_almost_full_get_interrupt_enabled(bool* is_enabled)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_ENABLE_ADDR, MAX30100_ENB_A_BIT_OFFSET, MAX30100_ENB_A_BIT_LENGTH);
	*is_enabled = !!val;
	return ret_state;
}

max30100_state_t max30100_temp_ready_interrupt(bool enable)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, enable?1:0, MAX30100_INTERRUPT_ENABLE_ADDR, MAX30100_ENB_TEP_RDY_BIT_OFFSET, MAX30100_ENB_TEP_RDY_BIT_LENGTH);
}

max30100_state_t max30100_temp_ready_get_interrupt_enabled(bool* is_enabled)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_ENABLE_ADDR, MAX30100_ENB_TEP_RDY_BIT_OFFSET, MAX30100_ENB_TEP_RDY_BIT_LENGTH);
	*is_enabled = !!val;
	return ret_state;
}

max30100_state_t max30100_hr_ready_interrupt(bool enable)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, enable?1:0, MAX30100_INTERRUPT_ENABLE_ADDR, MAX30100_ENB_HR_RDY_BIT_OFFSET, MAX30100_ENB_HR_RDY_BIT_LENGTH);
}

max30100_state_t max30100_hr_ready_get_interrupt_enabled(bool* is_enabled)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_INTERRUPT_ENABLE_ADDR, MAX30100_ENB_HR_RDY_BIT_OFFSET, MAX30100_ENB_HR_RDY_BIT_LENGTH);
	*is_enabled = !!val;
	return ret_state;
}

max30100_state_t max30100_set_fifo_write_pointer(uint8_t p_val)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, p_val, MAX30100_FIFO_WRITE_POINTER_ADDR, MAX30100_FIFO_WR_PTR_BIT_OFFSET, MAX30100_FIFO_WR_PTR_BIT_LENGTH);
}

max30100_state_t max30100_get_fifo_write_pointer(uint8_t * p_val)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_read(i2c, p_val, MAX30100_FIFO_WRITE_POINTER_ADDR, MAX30100_FIFO_WR_PTR_BIT_OFFSET, MAX30100_FIFO_WR_PTR_BIT_LENGTH);
}

max30100_state_t max30100_set_fifo_read_pointer(uint8_t p_val)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, p_val, MAX30100_FIFO_READ_POINTER_ADDR, MAX30100_FIFO_RD_PTR_BIT_OFFSET, MAX30100_FIFO_RD_PTR_BIT_LENGTH);
}

max30100_state_t max30100_get_fifo_read_pointer(uint8_t * p_val)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_read(i2c, p_val, MAX30100_FIFO_READ_POINTER_ADDR, MAX30100_FIFO_RD_PTR_BIT_OFFSET, MAX30100_FIFO_RD_PTR_BIT_LENGTH);
}

max30100_state_t max30100_get_fifo_overflow_counter(uint8_t* counter)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_read(i2c, counter, MAX30100_OVERFLOW_COUNTER_ADDR, MAX30100_OVF_COUNTER_BIT_OFFSET, MAX30100_OVF_COUNTER_BIT_LENGTH);
}

max30100_state_t max30100_fifo_read_next()
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t* data = (uint8_t*) &current_sample;
	max30100_state_t state = MAX30100_SUCCESS;
	//This could be done in burst mode...
	state = state == MAX30100_SUCCESS? max30100_register_value_read(i2c, &data[0], MAX30100_FIFO_DATA_REGISTER_ADDR, MAX30100_FIFO_DATA_BIT_OFFSET, MAX30100_FIFO_DATA_BIT_LENGTH) : state;
	state = state == MAX30100_SUCCESS? max30100_register_value_read(i2c, &data[1], MAX30100_FIFO_DATA_REGISTER_ADDR, MAX30100_FIFO_DATA_BIT_OFFSET, MAX30100_FIFO_DATA_BIT_LENGTH) : state;
	state = state == MAX30100_SUCCESS? max30100_register_value_read(i2c, &data[2], MAX30100_FIFO_DATA_REGISTER_ADDR, MAX30100_FIFO_DATA_BIT_OFFSET, MAX30100_FIFO_DATA_BIT_LENGTH) : state;
	state = state == MAX30100_SUCCESS? max30100_register_value_read(i2c, &data[3], MAX30100_FIFO_DATA_REGISTER_ADDR, MAX30100_FIFO_DATA_BIT_OFFSET, MAX30100_FIFO_DATA_BIT_LENGTH) : state;
	return state;
}

max30100_state_t max30100_get_current_fifo_sample_ir(uint16_t* sample)
{
	if (!i2c) return MAX30100_FAILURE;
	*sample = (uint16_t) (current_sample>>16);
	return MAX30100_SUCCESS;
}

max30100_state_t max30100_get_current_fifo_sample_red(uint16_t* sample)
{
	if (!i2c) return MAX30100_FAILURE;
	*sample = (uint16_t) (current_sample>>0);
	return MAX30100_SUCCESS;
}

max30100_state_t max30100_clear_fifo()
{
	if (!i2c) return MAX30100_FAILURE;
	max30100_state_t state = MAX30100_SUCCESS;
	//This could be done in burst mode...
	state = state == MAX30100_SUCCESS? max30100_set_fifo_write_pointer(0) : state;
	state = state == MAX30100_SUCCESS? max30100_set_fifo_read_pointer(0) : state;
	state = state == MAX30100_SUCCESS? max30100_register_value_write(i2c, 0, MAX30100_OVERFLOW_COUNTER_ADDR, MAX30100_OVF_COUNTER_BIT_OFFSET, MAX30100_OVF_COUNTER_BIT_LENGTH) : state;
	return state;
}

max30100_state_t max30100_get_fifo_samples_count(uint8_t* samples_count)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t write_pointer;
	uint8_t read_pointer;
	max30100_state_t state = MAX30100_SUCCESS;
	state = state == MAX30100_SUCCESS? max30100_get_fifo_read_pointer(&read_pointer) : state;
	state = state == MAX30100_SUCCESS? max30100_get_fifo_write_pointer(&write_pointer) : state;
	int8_t count = write_pointer - read_pointer;
	if(count < 0)
		count += MAX30100_FIFO_DEPTH;
	*samples_count = ((uint8_t) count);
	return state;
}

max30100_state_t max30100_shutdown_mode(bool shdn)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, shdn?1:0, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_SHDN_BIT_OFFSET, MAX30100_SHDN_BIT_LENGTH);
}

max30100_state_t max30100_get_shutdown_mode_enabled(bool* shdn)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_SHDN_BIT_OFFSET, MAX30100_SHDN_BIT_LENGTH);
	*shdn = !!val;
	return ret_state;
}

max30100_state_t max30100_reset()
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, 1, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_RESET_BIT_OFFSET, MAX30100_RESET_BIT_LENGTH);
}

max30100_state_t max30100_get_reset_ready(bool* ready)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_RESET_BIT_OFFSET, MAX30100_RESET_BIT_LENGTH);
	*ready = !val;
	return ret_state;
}

max30100_state_t max30100_wait_reset_ready()
{
	if (!i2c) return MAX30100_FAILURE;
	max30100_state_t state = MAX30100_SUCCESS;
	bool ready = false;
	while(state == MAX30100_SUCCESS && !ready)
		state = max30100_get_reset_ready(&ready);
	return state;
}

max30100_state_t max30100_trigger_temp_read()
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, 1, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_TEMP_EN_BIT_OFFSET, MAX30100_TEMP_EN_BIT_LENGTH);
}

max30100_state_t max30100_get_temperature_ready(bool* ready)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_TEMP_EN_BIT_OFFSET, MAX30100_TEMP_EN_BIT_LENGTH);
	*ready = !val;
	return ret_state;
}
max30100_state_t max30100_wait_temperature_ready()
{
	if (!i2c) return MAX30100_FAILURE;
	max30100_state_t state = MAX30100_SUCCESS;
	bool ready = false;
	while(state == MAX30100_SUCCESS && !ready)
		state = max30100_get_temperature_ready(&ready);
	return state;
}

max30100_state_t max30100_set_operating_mode(max30100_mode_t mode)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, (uint8_t) mode, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_MODE_BIT_OFFSET, MAX30100_MODE_BIT_LENGTH);
}

max30100_state_t max30100_get_operating_mode(max30100_mode_t* mode)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_MODE_CONFIGURATION_ADDR, MAX30100_MODE_BIT_OFFSET, MAX30100_MODE_BIT_LENGTH);
	*mode = (max30100_mode_t) val;
	return ret_state;
}

max30100_state_t max30100_spo2_high_resolution(bool hi_res)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, hi_res?1:0, MAX30100_SPO2_CONFIGURATION_ADDR, MAX30100_SPO2_HI_RES_EN_BIT_OFFSET, MAX30100_SPO2_HI_RES_EN_BIT_LENGTH);
}

max30100_state_t max30100_spo2_get_high_resolution_enabled(bool* hi_res)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	return max30100_register_value_read(i2c, &val, MAX30100_SPO2_CONFIGURATION_ADDR, MAX30100_SPO2_HI_RES_EN_BIT_OFFSET, MAX30100_SPO2_HI_RES_EN_BIT_LENGTH);
	*hi_res = !!val;
}

max30100_state_t max30100_spo2_set_sample_rate(max30100_spo2_sr_t sr)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, (uint8_t) sr, MAX30100_SPO2_CONFIGURATION_ADDR, MAX30100_SPO2_SR_BIT_OFFSET, MAX30100_SPO2_SR_BIT_LENGTH);
}

max30100_state_t max30100_spo2_get_sample_rate(max30100_spo2_sr_t* sr)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_SPO2_CONFIGURATION_ADDR, MAX30100_SPO2_SR_BIT_OFFSET, MAX30100_SPO2_SR_BIT_LENGTH);
	*sr = (max30100_spo2_sr_t) val;
	return ret_state;
}

max30100_state_t max30100_set_led_pulse_width(max30100_led_pw_t pw)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, (uint8_t) pw, MAX30100_SPO2_CONFIGURATION_ADDR, MAX30100_LED_PW_BIT_OFFSET, MAX30100_LED_PW_BIT_LENGTH);
}

max30100_state_t max30100_get_led_pulse_width(max30100_led_pw_t* pw)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_SPO2_CONFIGURATION_ADDR, MAX30100_LED_PW_BIT_OFFSET, MAX30100_LED_PW_BIT_LENGTH);
	*pw = (max30100_spo2_sr_t) val;
	return ret_state;
}

max30100_state_t max30100_set_red_led_current(max30100_led_cc_t cc)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, (uint8_t) cc, MAX30100_LED_CONFIGURATION_ADDR, MAX30100_RED_PA_BIT_OFFSET, MAX30100_RED_PA_BIT_LENGTH);
}

max30100_state_t max30100_get_red_led_current(max30100_led_cc_t* cc)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_LED_CONFIGURATION_ADDR, MAX30100_RED_PA_BIT_OFFSET, MAX30100_RED_PA_BIT_LENGTH);
	*cc = (max30100_led_cc_t) val;
	return ret_state;
}

max30100_state_t max30100_set_ir_led_current(max30100_led_cc_t cc)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_write(i2c, (uint8_t) cc, MAX30100_LED_CONFIGURATION_ADDR, MAX30100_IR_PA_BIT_OFFSET, MAX30100_IR_PA_BIT_LENGTH);
}

max30100_state_t max30100_get_ir_led_current(max30100_led_cc_t* cc)
{
	if (!i2c) return MAX30100_FAILURE;
	uint8_t val;
	max30100_state_t ret_state = max30100_register_value_read(i2c, &val, MAX30100_LED_CONFIGURATION_ADDR, MAX30100_IR_PA_BIT_OFFSET, MAX30100_IR_PA_BIT_LENGTH);
	*cc = (max30100_led_cc_t) val;
	return ret_state;
}

max30100_state_t max30100_get_temperature_c(float* temp)
{
	if (!i2c) return MAX30100_FAILURE;
	//This could be done in burst mode...
	int8_t t_int;
	uint8_t t_frac;
	max30100_state_t ret_state = max30100_register_value_read(i2c, (uint8_t*) &t_int, MAX30100_TEMP_INTEGER_ADDR, MAX30100_TINT_BIT_OFFSET, MAX30100_TINT_BIT_LENGTH);
	ret_state = ret_state == MAX30100_SUCCESS? max30100_register_value_read(i2c, &t_frac, MAX30100_TEMP_FRACTION_ADDR, MAX30100_TFRAC_BIT_OFFSET, MAX30100_TFRAC_BIT_LENGTH) : ret_state;
	*temp = MAX30100_FORM_TEMP_C(t_int,t_frac);
	return ret_state;
}

max30100_state_t max30100_get_revision_id(uint8_t* rev_id)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_read(i2c, rev_id, MAX30100_REVISION_ID_ADDR, MAX30100_REV_ID_BIT_OFFSET, MAX30100_REV_ID_BIT_LENGTH);
}

max30100_state_t max30100_get_part_id(uint8_t* part_id)
{
	if (!i2c) return MAX30100_FAILURE;
	return max30100_register_value_read(i2c, part_id, MAX30100_PART_ID_ADDR, MAX30100_PART_ID_BIT_OFFSET, MAX30100_PART_ID_BIT_LENGTH);
}
