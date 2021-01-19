/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    heartbeat.c
 * @brief   Application entry point.
 */

/* Default includes. */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* other includes. */
#include "drv/max30102.h"
#include <drv/rf_spo2_algorithm.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_A_IRQn I2C0_IRQn
#define UART_A_IRQn UART0_RX_TX_IRQn
/* Priorities at which the tasks are created.  */
#define mainEXAMPLE_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Application API */
static void example_task(void *pvParameters);
static void setup_max30102();
static void error_trap();
static void setup_max30205();
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * @brief   Application entry point.
 */

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif
    NVIC_SetPriority(I2C_A_IRQn, 5);
    NVIC_SetPriority(UART_A_IRQn, 5);
    NVIC_SetPriority(PORTB_IRQn, 4);


	/* RTOS Init Tasks. */
	if (xTaskCreate(example_task, "example_task",
	configMINIMAL_STACK_SIZE + 166, NULL, mainEXAMPLE_TASK_PRIORITY, NULL) != pdPASS) {
		PRINTF("Example task creation failed!.\r\n");
		while (1)
			;
	}
	else {
		setvbuf (stdout, NULL, _IONBF, 0);
//		PRINTF("Empezo\n");
	}
	vTaskStartScheduler();
	for (;;)
		;
}

//static float dummy;
static max30102_interrupt_status_t max30102_interrupts;

static uint32_t ir_led_samples[RF_SAMPLES + RF_SAMPLES_MARGIN];
static uint32_t red_led_samples[RF_SAMPLES + RF_SAMPLES_MARGIN];
char samp1[9];
char samp2[9];
char hr[5];
char spo2[5];

static uint32_t curr_buffer_n_samples = 0;

static int32_t curr_heart_rate = 0;
static int8_t curr_hr_valid = 0;
static float curr_spo2 = 0;
static int8_t curr_spo2_valid = 0;
static float curr_ratio = 0;
static float curr_correl = 0;

static volatile bool interrupt_flag = false;

static void setup_max30102(){
	NVIC_EnableIRQ(PORTB_IRQn);

	max30102_state_t state = max30102_init(MAX30102_SPO2_MODE);
	if(state == MAX30102_FAILURE) {
		PRINTF("MAX30102 INITIALIZATION ERROR\n");
		error_trap();
	}

	state = max30102_set_led_current(0x24,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_1);
	if(state == MAX30102_FAILURE) {
		PRINTF("MAX30102 LED1 ERROR\n");
		error_trap();
	}

	state = max30102_set_led_current(0x24,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_2);
	if(state == MAX30102_FAILURE){
		PRINTF("MAX30102 LED2 ERROR\n");
		error_trap();
	}

	max30102_fifo_configuration_t fifo_conf;
	fifo_conf.val = 0;
	fifo_conf.fifo_a_full = 0xF;
	fifo_conf.fifo_roll_over_en = true;
	fifo_conf.smp_ave = MAX30102_SMP_AVE_4;
	state = max30102_set_fifo_config(&fifo_conf);
	if(state == MAX30102_FAILURE) {
		PRINTF("MAX30102 FIFO CONFIG ERROR\n");
		error_trap();
	}
	max30102_spo2_configuration_t spo2_conf;
	spo2_conf.val = 0;
	spo2_conf.led_pw = MAX30102_LED_PW_411US_ADC_18_BITS;
	spo2_conf.spo2_sr = MAX30102_SPO2_SAMPLE_RATE_100HZ;
	spo2_conf.spo2_adc_rge = MAX30102_SPO2_ADC_RESOLUTION_4096NA;
	state = max30102_set_spo2_config(&spo2_conf);
	if(state == MAX30102_FAILURE) {
		PRINTF("MAX30102 SPO2 CONFIG ERROR\n");
		error_trap();
	}
	memset(&max30102_interrupts, 0, sizeof(max30102_interrupts));
}

static void handle_max_interrupts(){
	int counter = 0;
	while(true){
		while(interrupt_flag || !GPIO_PinRead(GPIOB, BOARD_MAX30102_INT_PIN_PIN)){
			max30102_get_interrupt_status(&max30102_interrupts);
			interrupt_flag = false;

			if(max30102_interrupts.pwr_rdy){
				max30102_interrupts.pwr_rdy = false;
				// no action necessary as the pwr_rdy flag is cleared when read.
				// "The interrupts are cleared whenever the interrupt status register is read,
				// or when the register that triggered the interrupt is read".
			}
			if(max30102_interrupts.ppg_rdy){
				max30102_interrupts.ppg_rdy = false;

//				uint8_t n_available_samples = max30102_get_num_available_samples();
				uint8_t n_available_samples = 1;
				uint32_t prev_buffer_n_samples = curr_buffer_n_samples;
				curr_buffer_n_samples += max30102_read_n_samples(n_available_samples,
						&ir_led_samples[curr_buffer_n_samples], &red_led_samples[curr_buffer_n_samples]);

				for(int i = prev_buffer_n_samples; i < curr_buffer_n_samples; i++){
					itoa((int) ir_led_samples[i], samp1, 10);
					itoa((int) red_led_samples[i], samp2, 10);

					UART_RTOS_Send(&UART0_rtos_handle, (uint8_t *)samp1, 7);
					UART_RTOS_Send(&UART0_rtos_handle, ",", 1);
					UART_RTOS_Send(&UART0_rtos_handle, (uint8_t *)samp2, 7);
					UART_RTOS_Send(&UART0_rtos_handle, "\n", 1);
				}

				//is the buffer full ? (should the accumulated samples be processed?)
				if(curr_buffer_n_samples >= RF_SAMPLES){
					counter++;
					if(counter > 5){
						//the red led is switched with the ir led on the board.
						rf_heart_rate_and_oxygen_saturation(red_led_samples, RF_SAMPLES, ir_led_samples,
							&curr_spo2, &curr_spo2_valid, &curr_heart_rate, &curr_hr_valid, &curr_ratio, &curr_correl);
					}
					if(curr_hr_valid && curr_spo2_valid){
						itoa((int) curr_heart_rate, hr, 10);
						itoa((int) curr_spo2, spo2, 10);

//						UART_RTOS_Send(&UART0_rtos_handle, (uint8_t *)hr, 5);
//						UART_RTOS_Send(&UART0_rtos_handle, ",", 1);
//						UART_RTOS_Send(&UART0_rtos_handle, (uint8_t *)spo2, 5);
//						UART_RTOS_Send(&UART0_rtos_handle, "\n", 1);
					}
					curr_buffer_n_samples = 0;

				}
			}
			if(max30102_interrupts.alc_ovf){
				max30102_interrupts.alc_ovf = false;

			}
			if(max30102_interrupts.die_temp_rdy){
				max30102_interrupts.die_temp_rdy = false;

			}

			if(max30102_interrupts.a_full){
				max30102_interrupts.a_full = false;
			}

		}
		//vTaskSuspend(NULL);
	}
}


static void example_task(void *pvParameters) {

	setup_max30102();
	max30102_trigger_spo2_reads();
	handle_max_interrupts();

	for (;;) {
		vTaskSuspend(NULL);
	}

}

/* PORTB_IRQn interrupt handler */
void GPIOB_IRQHANDLER(void) {
  /* Get pin flags */
  uint32_t pin_flags = GPIO_PortGetInterruptFlags(GPIOB);

  for(int i = 0; i < sizeof(pin_flags)*8; i++){
	  if(pin_flags & (1 << i)){
		  if(i == BOARD_MAX30102_INT_PIN_PIN){
			  interrupt_flag = true;
			  break;
		  }
	  }
  }

  /* Clear pin flags */
  GPIO_PortClearInterruptFlags(GPIOB, pin_flags);

  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


static void error_trap(){
	PRINTF("ERROR - TRAP");
	while(1);

}
