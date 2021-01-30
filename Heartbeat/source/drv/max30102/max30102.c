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
#include "drv/i2c/i2c.h"

#define MAX30102_TFRAC_STEP_C	1.0/16
#define MAX30102_TINT_SETP_C	1
#define MAX30102_CALCULATE_TEMP_C(tint,tfrac) ((float)(MAX30102_TINT_SETP_C*((int8_t)(tint)) + MAX30102_TFRAC_STEP_C*((uint8_t)(tfrac))))
#define MAX30102_SAMPLE_N_BYTES 3

i2c_handle_t* i2c = NULL;

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

	uint8_t data[1]={0x00};
	i2c_write_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_ENABLE_2_ADDR, 1, data);
	i2c_write_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_WRITE_POINTER_ADDR, 1, data);
	i2c_write_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_OVERFLOW_COUNTER_ADDR, 1, data);
	i2c_write_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_READ_POINTER_ADDR, 1, data);

	max30102_set_mode(initial_mode);

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
		*temp = MAX30102_CALCULATE_TEMP_C(integer,frac);
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

max30102_state_t max30102_set_die_temp_rdy_en(bool die_tmp_rdy_en){
	if(!i2c) return MAX30102_FAILURE;

	max30102_interrupt_enable_t enables;
	memset(&enables, 0, sizeof(enables));
	enables.die_temp_rdy_en = die_tmp_rdy_en;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_DIE_TEMP_EN_BIT_START, MAX30102_DIE_TEMP_EN_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_ENABLE_2_ADDR, mask, enables.byte[0])?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}
max30102_state_t max30102_set_alc_ovf_en(bool alc_ovf_en){
	if(!i2c) return MAX30102_FAILURE;

	max30102_interrupt_enable_t enables;
	memset(&enables, 0, sizeof(enables));
	enables.alc_ovf_en = alc_ovf_en;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_ALC_OVF_BIT_START, MAX30102_ALC_OVF_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_ENABLE_1_ADDR, mask, enables.byte[1])?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}
max30102_state_t max30102_set_ppg_rdy_en(bool ppg_rdy_en){
	if(!i2c) return MAX30102_FAILURE;

	max30102_interrupt_enable_t enables;
	memset(&enables, 0, sizeof(enables));
	enables.ppg_rdy_en = ppg_rdy_en;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_ppg_RDY_BIT_START, MAX30102_ppg_RDY_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_ENABLE_1_ADDR, mask, enables.byte[1])?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}
max30102_state_t max30102_set_a_full_en(bool a_full_en){
	if(!i2c) return MAX30102_FAILURE;

	max30102_interrupt_enable_t enables;
	memset(&enables, 0, sizeof(enables));
	enables.a_full_en = a_full_en;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_A_FULL_BIT_START, MAX30102_A_FULL_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_ENABLE_1_ADDR, mask, enables.byte[1])?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_fifo_config(max30102_fifo_configuration_t *config){
	if(!i2c) return MAX30102_FAILURE;

	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_CONFIGURATION_ADDR, config->val)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_fifo_a_full(uint8_t fifo_a_full){
	if(!i2c) return MAX30102_FAILURE;
	max30102_fifo_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.fifo_a_full = fifo_a_full;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_FIFO_A_FULL_BIT_START, MAX30102_FIFO_A_FULL_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_CONFIGURATION_ADDR, mask, new_conf.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_fifo_roll_over_en(bool roll_over_en){
	if(!i2c) return MAX30102_FAILURE;
	max30102_fifo_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.fifo_roll_over_en = roll_over_en;

	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_FIFO_ROLL_OVER_EN_BIT_START, MAX30102_FIFO_ROLL_OVER_EN_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_CONFIGURATION_ADDR, mask, new_conf.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_fifo_smp_ave(max30102_smp_ave_t smp_ave){
	if(!i2c) return MAX30102_FAILURE;
	max30102_fifo_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.smp_ave = smp_ave;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_SMP_AVE_BIT_LENGTH, MAX30102_SMP_AVE_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_CONFIGURATION_ADDR, mask, new_conf.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_spo2_config(max30102_spo2_configuration_t *config){
	if(!i2c) return MAX30102_FAILURE;
	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_SPO2_CONFIGURATION_ADDR, config->val)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_spo2_adc_rge(max30102_spo2_adc_resolution_t spo2_adc_rge){
	if(!i2c) return MAX30102_FAILURE;
	max30102_spo2_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.spo2_adc_rge = spo2_adc_rge;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_SPO2_ADC_RGE_BIT_START, MAX30102_SPO2_ADC_RGE_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_SPO2_CONFIGURATION_ADDR, mask, new_conf.val)?
						MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_sr(max30102_spo2_sample_rate_t spo2_sample_rate){
	if(!i2c) return MAX30102_FAILURE;
	max30102_spo2_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.spo2_sr = spo2_sample_rate;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_SPO2_SR_BIT_START, MAX30102_SPO2_SR_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_SPO2_CONFIGURATION_ADDR, mask, new_conf.val)?
						MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_led_pw(max30102_led_pw_t led_pw){
	if(!i2c) return MAX30102_FAILURE;
	max30102_spo2_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.led_pw = led_pw;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_LED_PW_BIT_START, MAX30102_LED_PW_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_SPO2_CONFIGURATION_ADDR, mask, new_conf.val)?
						MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_led_current(uint8_t curr_lvl, max30102_addr_t led_addr){
	if(!i2c) return MAX30102_FAILURE;
	if(led_addr != MAX30102_LED_PULSE_AMPLITUDE_ADDR_1 && led_addr != MAX30102_LED_PULSE_AMPLITUDE_ADDR_2) return MAX30102_FAILURE;
	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, led_addr, curr_lvl) ? MAX30102_SUCCESS : MAX30102_FAILURE;
}

uint8_t max30102_read_sample(uint32_t *ir_data, uint32_t *red_data){

	if(!i2c) return 0;
	uint32_t un_temp = 0;
	uint8_t data[6];

	if(!i2c_read_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_FIFO_DATA_REGISTER_ADDR, 6, data)){
		return 0;
	}
	else{
		*red_data = 0;
		*ir_data = 0;

		un_temp = (unsigned char) data[0];
		un_temp<<=16;
		*red_data += un_temp;
		un_temp = (unsigned char) data[1];
		un_temp <<= 8;
		*red_data+=un_temp;
		un_temp=(unsigned char) data[2];
		*red_data+=un_temp;

		un_temp=(unsigned char) data[3];
		un_temp<<=16;
		*ir_data+=un_temp;
		un_temp=(unsigned char) data[4];
		un_temp<<=8;
		*ir_data+=un_temp;
		un_temp=(unsigned char) data[5];
		*ir_data+=un_temp;

		*red_data&=0x03FFFF;  //Mask MSB [23:18]
		*ir_data&=0x03FFFF;  //Mask MSB [23:18]
	}

	return 1;
}

max30102_state_t max30102_trigger_spo2_reads(){
	if(!i2c) return MAX30102_FAILURE;
	return max30102_set_ppg_rdy_en(true);
}


max30102_state_t max30102_get_interrupt_status(max30102_interrupt_status_t *status){
	if(!i2c) return MAX30102_FAILURE;

	max30102_state_t state;
	state = i2c_read_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_STATUS_1_ADDR, &(status->byte[1]))?
			MAX30102_SUCCESS : MAX30102_FAILURE;
	if(state == MAX30102_SUCCESS)
		state = i2c_read_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_INTERRUPT_STATUS_2_ADDR, &(status->byte[0])) ?
				MAX30102_SUCCESS: MAX30102_FAILURE;
	return state;
}


max30102_state_t max30102_set_mode_config(max30102_mode_configuration_t *config){
	if(!i2c) return MAX30102_FAILURE;

	return i2c_write_byte_addr8(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, config->val)?
					MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_mode(max30102_mode_t mode){
	if(!i2c) return MAX30102_FAILURE;

	max30102_mode_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.mode = mode;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_MODE_BIT_START, MAX30102_MODE_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, mask, new_conf.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}

max30102_state_t max30102_set_shdn(bool shdn){
	if(!i2c) return MAX30102_FAILURE;

	max30102_mode_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.shdn = shdn;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_SHDN_BIT_START, MAX30102_SHDN_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, mask, new_conf.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}


max30102_state_t max30102_set_reset(bool reset){
	if(!i2c) return MAX30102_FAILURE;

	max30102_mode_configuration_t new_conf;
	memset(&new_conf, 0, sizeof(new_conf));
	new_conf.reset = reset;
	uint8_t mask = MAX30102_MASK_GENERATE(MAX30102_RESET_BIT_START, MAX30102_RESET_BIT_LENGTH);
	return i2c_write_byte_addr8_mask(i2c, MAX30102_I2C_ADDRESS, MAX30102_MODE_CONFIGURATION_ADDR, mask, new_conf.val)?
				MAX30102_SUCCESS : MAX30102_FAILURE;
}