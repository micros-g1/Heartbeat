/*
 * max30102_registers.h
 *
 *  Created on: 29 Oct 2020
 *      Author: grein
 */

#ifndef DRV_MAX30102_REGISTERS_H_
#define DRV_MAX30102_REGISTERS_H_

#include <stdint.h>

// Information from 'MAX30102 High-Sensitivity Pulse Oximeter and Heart-Rate Sensor for Wearable Health' Rev. 1 10/18
// https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf

//WRITE ADDRESS: 0xAE = 10101110, READ ADDRESS: 0xAF= 10101111
//ADDR: 1010111x: 0101 0111 = 0x57
#define MAX30102_I2C_ADDRESS 0x57
#define MAX30102_PART_ID 0x15

//Register Map
typedef enum
{
	MAX30102_INTERRUPT_STATUS_1_ADDR = 0x00,
	MAX30102_INTERRUPT_STATUS_2_ADDR = 0x01,
	MAX30102_INTERRUPT_ENABLE_1_ADDR = 0x02,
	MAX30102_INTERRUPT_ENABLE_2_ADDR = 0x03,
	MAX30102_FIFO_WRITE_POINTER_ADDR = 0x04,
	MAX30102_OVERFLOW_COUNTER_ADDR = 0x05,
	MAX30102_FIFO_READ_POINTER_ADDR = 0x06,
	MAX30102_FIFO_DATA_REGISTER_ADDR = 0x07,
	MAX30102_FIFO_CONFIGURATION_ADDR = 0x08,
	MAX30102_MODE_CONFIGURATION_ADDR = 0x09,
	MAX30102_SPO2_CONFIGURATION_ADDR = 0x0A,
	MAX30102_LED_PULSE_AMPLITUDE_ADDR_1 = 0x0C,
	MAX30102_LED_PULSE_AMPLITUDE_ADDR_2 = 0x0D,
	MAX30102_MULTI_LED_MODE_CONTROL_REGISTER_1_ADDR = 0x11,
	MAX30102_MULTI_LED_MODE_CONTROL_REGISTER_2_ADDR = 0x12,
	MAX30102_DIE_TEMPERATURE_INTEGER_ADDR = 0x1F,
	MAX30102_DIE_TEMPERATURE_FRACTION_ADDR = 0x20,
	MAX30102_DIE_TEMPERATURE_CONFIG_ADDR = 0x21,
	MAX30102_REVISION_ID_ADDR = 0xFE,
	MAX30102_PART_ID_ADDR = 0xFF
}max30102_addr_t;

#define MAX30102_MASK_GENERATE(BITSTART,LENGTH)  ( ( (1 << (LENGTH)) - 1 ) << (  (BITSTART) + 1 - (LENGTH)) )
#define MAX30102_APPLY_BITSTART_LENGTH(DATA,BITSTART,LENGTH)  ( ( (DATA) << (  (BITSTART) + 1 - (LENGTH)) ) & MAX30102_MASK_GENERATE(BITSTART,LENGTH) )
#define MAX30102_RECOVER_BITSTART_LENGTH(DATA,BITSTART,LENGTH) ( ( (DATA) & MAX30102_MASK_GENERATE(BITSTART,LENGTH) ) >> (  (BITSTART) + 1 - (LENGTH)) )

//Interrupt Status 1
#define MAX30102_A_FULL_BIT_START 7
#define MAX30102_A_FULL_BIT_LENGTH 1
#define MAX30102_ppg_RDY_BIT_START 6
#define MAX30102_ppg_RDY_BIT_LENGTH 1
#define MAX30102_ALC_OVF_BIT_START 5
#define MAX30102_ALC_OVF_BIT_LENGTH 1
#define MAX30102_PWR_RDY_BIT_START 0
#define MAX30102_PWR_RDY_BIT_LENGTH 1
//Interrupt Status 2
#define MAX30102_DIE_TMP_RDY_BIT_START 1
#define MAX30102_DIE_TMP_RDY_BIT_LENGTH 1
//Interrupt Status
typedef union
{
	struct{
		unsigned int : 1;
		unsigned int die_temp_rdy : 1;
		unsigned int : 6;
		unsigned int pwr_rdy : 1;
		unsigned int : 4;
		unsigned int alc_ovf : 1;
		unsigned int ppg_rdy : 1;
		unsigned int a_full : 1;
	};
	uint8_t byte[2];
	uint16_t val;
}__attribute__((__packed__, aligned(1))) max30102_interrupt_status_t;
//Interrupt Enable 1
#define MAX30102_A_FULL_EN_BIT_START 7
#define MAX30102_A_FULL_EN_BIT_LENGTH 1
#define MAX30102_ppg_RDY_EN_BIT_START 6
#define MAX30102_ppg_RDY_EN_BIT_LENGTH 1
#define MAX30102_ALC_OVF_EN_BIT_START 5
#define MAX30102_ALC_OVF_EN_BIT_LENGTH 1
//Interrupt Enable 2
#define MAX30102_DIE_TMP_RDY_EN_BIT_START 1
#define MAX30102_DIE_TMP_RDY_EN_BIT_LENGTH 1
//Interrupt Enable
typedef union
{
	struct{
		unsigned int : 1;
		unsigned int die_temp_rdy_en : 1;
		unsigned int : 11;
		unsigned int alc_ovf_en : 1;
		unsigned int ppg_rdy_en : 1;
		unsigned int a_full_en : 1;
	};
	uint8_t byte[2];
	uint16_t val;
}__attribute__((__packed__, aligned(1))) max30102_interrupt_enable_t;
//FIFO Write Pointer
#define MAX30102_FIFO_WR_PTR_BIT_START 4
#define MAX30102_FIFO_WR_PTR_BIT_LENGTH 5
//Overflow Counter
#define MAX30102_OVF_COUNTER_BIT_START 4
#define MAX30102_OVF_COUNTER_BIT_LENGTH 5
//FIFO Read Counter
#define MAX30102_FIFO_RD_PTR_BIT_START 4
#define MAX30102_FIFO_RD_PTR_BIT_LENGTH 5
//FIFO Configuration
#define MAX30102_SMP_AVE_BIT_START 7
#define MAX30102_SMP_AVE_BIT_LENGTH 3
#define MAX30102_FIFO_ROLL_OVER_EN_BIT_START 4
#define MAX30102_FIFO_ROLL_OVER_EN_BIT_LENGTH 1
#define MAX30102_FIFO_A_FULL_BIT_START 3
#define MAX30102_FIFO_A_FULL_BIT_LENGTH 4
typedef enum
{
	MAX30102_SMP_AVE_1 = 0b000,
	MAX30102_SMP_AVE_2 = 0b001,
	MAX30102_SMP_AVE_4 = 0b010,
	MAX30102_SMP_AVE_8 = 0b011,
	MAX30102_SMP_AVE_16 = 0b100,
	MAX30102_SMP_AVE_32 = 0b101,
}max30102_smp_ave_t;
typedef union
{
	struct{
		unsigned int fifo_a_full : 4;
		unsigned int fifo_roll_over_en : 1;
		max30102_smp_ave_t smp_ave : 3;
	};
	uint8_t byte[1];
	uint8_t val;
}__attribute__((__packed__, aligned(1))) max30102_fifo_configuration_t;
//Mode Configuration
#define MAX30102_SHDN_BIT_START 7
#define MAX30102_SHDN_BIT_LENGTH 1
#define MAX30102_RESET_BIT_START 7
#define MAX30102_RESET_BIT_LENGTH 1
#define MAX30102_MODE_BIT_START 2
#define MAX30102_MODE_BIT_LENGTH 3
typedef enum
{
	MAX30102_HEART_RATE_MODE = 0b010,
	MAX30102_SPO2_MODE = 0b011
}max30102_mode_t;

typedef union
{
	struct
	{
		max30102_mode_t mode : 3;
		unsigned int : 3;
		unsigned int reset : 1;
		unsigned int shdn : 1;
	};
	uint8_t byte[1];
	uint8_t val;
}__attribute__((__packed__, aligned(1))) max30102_mode_configuration_t;
//SPO2 Configuration
#define MAX30102_SPO2_ADC_RGE_BIT_START 6
#define MAX30102_SPO2_ADC_RGE_BIT_LENGTH 2
#define MAX30102_SPO2_SR_BIT_START 4
#define MAX30102_SPO2_SR_BIT_LENGTH 3
#define MAX30102_LED_PW_BIT_START 1
#define MAX30102_LED_PW_BIT_LENGTH 2
typedef enum
{
	MAX30102_SPO2_ADC_RESOLUTION_2048NA = 0b00,
	MAX30102_SPO2_ADC_RESOLUTION_4096NA = 0b01,
	MAX30102_SPO2_ADC_RESOLUTION_8192NA = 0b10,
	MAX30102_SPO2_ADC_RESOLUTION_16384NA = 0b11
}max30102_spo2_adc_resolution_t;
typedef enum
{
	MAX30102_SPO2_SAMPLE_RATE_50HZ = 0b000,
	MAX30102_SPO2_SAMPLE_RATE_100HZ = 0b001,
	MAX30102_SPO2_SAMPLE_RATE_200HZ = 0b010,
	MAX30102_SPO2_SAMPLE_RATE_400HZ = 0b011,
	MAX30102_SPO2_SAMPLE_RATE_800HZ = 0b100,
	MAX30102_SPO2_SAMPLE_RATE_1000HZ = 0b101,
	MAX30102_SPO2_SAMPLE_RATE_1600HZ = 0b110,
	MAX30102_SPO2_SAMPLE_RATE_3200HZ = 0b111,
}max30102_spo2_sample_rate_t;
typedef enum
{
	MAX30102_LED_PW_69US_ADC_15_BITS = 0b00,
	MAX30102_LED_PW_118US_ADC_16_BITS = 0b01,
	MAX30102_LED_PW_215US_ADC_17_BITS = 0b10,
	MAX30102_LED_PW_411US_ADC_18_BITS = 0b11,
}max30102_led_pw_t;
typedef union
{
	struct
	{
		max30102_led_pw_t led_pw : 2;
		max30102_spo2_sample_rate_t spo2_sr : 3;
		max30102_spo2_adc_resolution_t spo2_adc_rge : 2;
		unsigned int : 1;
	};
	uint8_t byte[1];
	uint8_t val;
}__attribute__((__packed__, aligned(1))) max30102_spo2_configuration_t;
//LED Pulse Amplitude
#define MAX30102_LED_PA_BIT_START 7
#define MAX30102_LED_PA_BIT_LENGTH 8
//Multi-LED Mode Control Registers
#define MAX30102_SLOT1_BIT_START 2
#define MAX30102_SLOT1_BIT_LENGTH 3
#define MAX30102_SLOT2_BIT_START 6
#define MAX30102_SLOT2_BIT_LENGTH 3
#define MAX30102_SLOT3_BIT_START 2
#define MAX30102_SLOT3_BIT_LENGTH 3
#define MAX30102_SLOT4_BIT_START 6
#define MAX30102_SLOT4_BIT_LENGTH 3
typedef enum
{
	MAX30102_TIME_SLOT_DISABLED = 0b000,
	MAX30102_SLOT_LED_RED = 0b001,
	MAX30102_SLOT_LED_RIR = 0b010,
	MAX30102_SLOT_LED_NONE = 0b000,
}max30102_slot_t;
typedef union
{
	struct
	{
		max30102_slot_t slot3 : 3;
		unsigned int : 1;
		max30102_slot_t slot4 : 3;
		unsigned int : 1;
		max30102_slot_t slot1 : 3;
		unsigned int : 1;
		max30102_slot_t slot2 : 3;
		unsigned int : 1;
	};
	uint8_t byte[2];
	uint8_t val;
}__attribute__((__packed__, aligned(1))) max30102_multiled_control_register_t;
//Temperature
#define MAX30102_DIE_TEMP_INTEGER_BIT_START 7
#define MAX30102_DIE_TEMP_INTEGER_LENGTH 8
#define MAX30102_DIE_TEMP_FRACTION_BIT_START 3
#define MAX30102_DIE_TEMP_FRACTION_LENGTH 4
#define MAX30102_DIE_TEMP_EN_BIT_START 0
#define MAX30102_DIE_TEMP_EN_BIT_LENGTH 1
#define MAX30102_DIE_TEMP_INTEGER_STEP_C 1
#define MAX30102_DIE_TEMP_FRAC_STEP_C 0.0625
#define MAX30102_TEMP_C(TIN, TFRAC) ((float)((int8_t)(TINT) + MAX30102_DIE_TEMP_FRAC_STEP_C*((uint8_t)TFRAC)))
//PART ID, REV
#define MAX30102_REVISION_ID_BIT_START 7
#define MAX30102_REVISION_ID_BIT_LENGTH 8
#define MAX30102_PART_ID_BIT_START 7
#define MAX30102_PART_ID_BIT_LENGTH 8

#endif /* DRV_MAX30102_REGISTERS_H_ */
