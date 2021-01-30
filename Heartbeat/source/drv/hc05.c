/*
 * hc05.c
 *
 *  Created on: 28 ene. 2021
 *      Author: lisan
 */
#include "board/peripherals.h"
#include "drivers/fsl_uart.h"

#include "hc05.h"

#define VCC_PIN	18
#define KEY_PIN	19



hc05_state_t hc05_init(){
	hc05_state_t success = HC05_SUCCESS;
	gpio_pin_config_t pin_config = {
			kGPIO_DigitalOutput,
			1
	};

	GPIO_PinInit(GPIOB, KEY_PIN, &pin_config);
	GPIO_PinInit(GPIOB, KEY_PIN, &pin_config);

	GPIO_PinWrite(GPIOB, VCC_PIN,0);
	GPIO_PinWrite(GPIOB, KEY_PIN, 1);
	GPIO_PinWrite(GPIOB, VCC_PIN,1);

	return success;
}
