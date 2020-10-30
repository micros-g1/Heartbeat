/*
 * max30100_registers.h
 *
 *  Created on: 29 Oct 2020
 *      Author: grein
 */

#ifndef DRV_MAX30100_REGISTERS_H_
#define DRV_MAX30100_REGISTERS_H_

#include <stdint.h>

// Information from 'MAX30100 Pulse oximeter and Heart-Rate Sensor IC for Wearable Health' datasheet Rev 0


//WRITE ADDRESS: 0xAE = 10101110, READ ADDRESS: 0xAF= 10101111
//ADDR: 1010111x: 0101 0111 = 0x57
#define MAX30100_I2C_ADDRESS 0x57
#define MAX30100_PART_ID 0x11

//Table 1. Register Maps and Descriptions
//STATUS
#define MAX30100_INTERRUPT_STATUS_ADDR 		0x00
#define MAX30100_INTERRUPT_ENABLE_ADDR 		0x01
//FIFO
#define MAX30100_FIFO_WRITE_POINTER_ADDR	0x02
#define MAX30100_OVERFLOW_COUNTER_ADDR		0x03
#define MAX30100_FIFO_READ_POINTER_ADDR		0x04
#define MAX30100_FIFO_DATA_REGISTER_ADDR	0x05
//CONFIGURATION
#define MAX30100_MODE_CONFIGURATION_ADDR	0x06
#define MAX30100_SPO2_CONFIGURATION_ADDR	0x07
#define MAX30100_LED_CONFIGURATION_ADDR		0x09
//TEMPERATURE
#define MAX30100_TEMP_INTEGER_ADDR			0x16
#define MAX30100_TEMP_FRACTION_ADDR			0x17
//PART ID
#define MAX30100_REVISION_ID_ADDR			0xFE
#define MAX30100_PART_ID_ADDR				0xFF



//INTERRUPT STATUS
#define MAX30100_FIFO_ALMOST_FULL_FLAG_BIT_OFFSET 7
#define MAX30100_FIFO_ALMOST_FULL_FLAG_BIT_LENGTH 1
#define MAX30100_TEMPERATURE_READY_FLAG_BIT_OFFSET 6
#define MAX30100_TEMPERATURE_READY_FLAG_BIT_LENGTH 1
#define MAX30100_HEARTRATE_DATA_READY_BIT_OFFSET 5
#define MAX30100_HEARTRATE_DATA_READY_BIT_LENGTH 1
#define MAX30100_SPO2_DATA_READY_BIT_OFFSET 4
#define MAX30100_SPO2_DATA_READY_BIT_LENGTH 1
#define MAX30100_POWER_READY_BIT_OFFSET 0
#define MAX30100_POWER_READY_BIT_LENGTH 1



//INTERRUPT ENABLE
#define MAX30100_ENB_A_BIT_OFFSET 7
#define MAX30100_ENB_A_BIT_LENGTH 1
#define MAX30100_ENB_TEP_RDY_BIT_OFFSET 6
#define MAX30100_ENB_TEP_RDY_BIT_LENGTH 1
#define MAX30100_ENB_HR_RDY_BIT_OFFSET 5
#define MAX30100_ENB_HR_RDY_BIT_LENGTH 1
#define MAX30100_ENB_SO2_RDY_BIT_OFFSET 4
#define MAX30100_ENB_SO2_RDY_BIT_LENGTH 1



//FIFO WRITE POINTER
#define MAX30100_FIFO_WR_PTR_BIT_OFFSET 0
#define MAX30100_FIFO_WR_PTR_BIT_LENGTH 4



//OVERFLOW COUNTER
#define MAX30100_OVF_COUNTER_BIT_OFFSET 0
#define MAX30100_OVF_COUNTER_BIT_LENGTH 4



//FIFO READ POINTER
#define MAX30100_FIFO_RD_PTR_BIT_OFFSET 0
#define MAX30100_FIFO_RD_PTR_BIT_LENGTH 4



//FIFO DATA
#define MAX30100_FIFO_DATA_BIT_OFFSET 0
#define MAX30100_FIFO_DATA_BIT_LENGTH 8



//MODE CONFIGURATION
#define MAX30100_SHDN_BIT_OFFSET 7
#define MAX30100_SHDN_BIT_LENGTH 1
#define MAX30100_RESET_BIT_OFFSET 6
#define MAX30100_RESET_BIT_LENGTH 1
#define MAX30100_TEMP_EN_BIT_OFFSET 3
#define MAX30100_TEMP_EN_BIT_LENGTH 1
#define MAX30100_MODE_BIT_OFFSET 0
#define MAX30100_MODE_BIT_LENGTH 3



//SPO2 CONFIGURATION
#define MAX30100_SPO2_HI_RES_EN_BIT_OFFSET 6
#define MAX30100_SPO2_HI_RES_EN_BIT_LENGTH 1
#define MAX30100_SPO2_SR_BIT_OFFSET 2
#define MAX30100_SPO2_SR_BIT_LENGTH 3
#define MAX30100_LED_PW_BIT_OFFSET 0
#define MAX30100_LED_PW_BIT_LENGTH 2



//LED CONFIGURATION
#define MAX30100_RED_PA_BIT_OFFSET 4
#define MAX30100_RED_PA_BIT_LENGTH 4
#define MAX30100_IR_PA_BIT_OFFSET 0
#define MAX30100_IR_PA_BIT_LENGTH 4



//TEMP INTEGER
#define MAX30100_TINT_BIT_OFFSET 0
#define MAX30100_TINT_BIT_LENGTH 8



//TEMP FRACTION
#define MAX30100_TFRAC_BIT_OFFSET 0
#define MAX30100_TFRAC_BIT_LENGTH 4



//REVISION ID
#define MAX30100_REV_ID_BIT_OFFSET 0
#define MAX30100_REV_ID_BIT_LENGTH 8



//PART ID
#define MAX30100_PART_ID_BIT_OFFSET 0
#define MAX30100_PART_ID_BIT_LENGTH 8



//Mode Control Table 3
typedef enum
{
	MAX30100_MODE_HR_ONLY_ENABLED = 0b010,
	MAX30100_MODE_SPO2_ENABLED = 0b011
}max30100_mode_t;

//SpO2 Sample Rate Control Table 4
typedef enum
{

	MAX30100_SP02_SR_50_HZ 		= 0b000,
	MAX30100_SP02_SR_100_HZ		= 0b001,
	MAX30100_SP02_SR_167_HZ		= 0b010,
	MAX30100_SP02_SR_200_HZ		= 0b011,
	MAX30100_SP02_SR_400_HZ		= 0b100,
	MAX30100_SP02_SR_600_HZ		= 0b101,
	MAX30100_SP02_SR_800_HZ		= 0b110,
	MAX30100_SP02_SR_1000_HZ	= 0b111
}max30100_spo2_sr_t;


//LED Pulse Width Control Table 5
typedef enum
{
	MAX30100_LED_PW_200_US_ADC_13_BITS 		= 0b00,
	MAX30100_LED_PW_400_US_ADC_14_BITS 		= 0b01,
	MAX30100_LED_PW_800_US_ADC_15_BITS 		= 0b10,
	MAX30100_LED_PW_1600_US_ADC_16_BITS 	= 0b11
}max30100_led_pw_t;

//:LED Current Control Table 6
typedef enum
{
	MAX30100_LED_CC_0m0A 		= 0b0000,
	MAX30100_LED_CC_4m4A 		= 0b0001,
	MAX30100_LED_CC_7m6A 		= 0b0010,
	MAX30100_LED_CC_11m0A 		= 0b0011,
	MAX30100_LED_CC_14m2A 		= 0b0100,
	MAX30100_LED_CC_17m4A 		= 0b0101,
	MAX30100_LED_CC_20m8A 		= 0b0110,
	MAX30100_LED_CC_24m0A 		= 0b0111,
	MAX30100_LED_CC_27m1A 		= 0b1000,
	MAX30100_LED_CC_30m6A 		= 0b1001,
	MAX30100_LED_CC_33m8A 		= 0b1010,
	MAX30100_LED_CC_37m0A 		= 0b1011,
	MAX30100_LED_CC_40m2A 		= 0b1100,
	MAX30100_LED_CC_43m6A 		= 0b1101,
	MAX30100_LED_CC_46m8A 		= 0b1110,
	MAX30100_LED_CC_50m0A 		= 0b1111
}max30100_led_cc_t;

#endif /* DRV_MAX30100_REGISTERS_H_ */
