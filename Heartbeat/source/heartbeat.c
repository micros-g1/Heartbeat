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

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_A_IRQn I2C0_IRQn

/* Priorities at which the tasks are created.  */
#define mainEXAMPLE_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Application API */
static void example_task(void *pvParameters);

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

static float dummy;
static void example_task(void *pvParameters) {

	max30102_state_t state = max30102_init(MAX30102_SPO2_MODE);
//	state = state == MAX30102_SUCCESS ? max30102_trigger_temp_read(): state;
//	state = state == MAX30102_SUCCESS ? max30102_wait_temp_read_ready(): state;
//	state = state == MAX30102_SUCCESS ? max30102_get_temperature_c(&dummy): state;

	max30102_state_t state3 = max30102_set_led_current(0xFF,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_1);
	max30102_state_t state4 = max30102_set_led_current(0xFF,  MAX30102_LED_PULSE_AMPLITUDE_ADDR_2);

	max30102_fifo_configuration_t fifo_conf;
	fifo_conf.val = 0;
	fifo_conf.fifo_a_full = 0xF;
	fifo_conf.fifo_roll_over_en = true;
	fifo_conf.smp_ave = MAX30102_SMP_AVE_1;
	max30102_state_t state1 = max30102_set_fifo_config(&fifo_conf);

	max30102_spo2_configuration_t spo2_conf;
	spo2_conf.val = 0;
	spo2_conf.led_pw = MAX30102_LED_PW_411US_ADC_18_BITS;
	spo2_conf.spo2_sr = MAX30102_SPO2_SAMPLE_RATE_100HZ;
	spo2_conf.spo2_adc_rge = MAX30102_SPO2_ADC_RESOLUTION_4096NA;
	max30102_state_t state2 = max30102_set_spo2_config(&spo2_conf);

	NVIC_EnableIRQ(PORTB_IRQn);

	max30102_trigger_spo2_read();

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
			  max30102_interrupt_status_t status;
			  max30102_get_interrupt_status(&status);
			  if(status.pwr_rdy){

			  }
			  if(status.ppg_rdy){
				  max30102_set_ppg_rdy_en(false);
				  max30102_set_a_full_en(false);
				  uint8_t n_available_samples = max30102_get_num_available_samples();
				  uint8_t m_read_samples = 0;
				  max30102_sample_t *samples = max30102_read_n_samples(n_available_samples, &m_read_samples);
				  for(int i =0; i < m_read_samples; i++)
					  PRINTF("%d , ", samples[i]);
				  max30102_set_ppg_rdy_en(true);
				  max30102_set_a_full_en(true);
			  }
			  if(status.alc_ovf){

			  }
			  if(status.die_temp_rdy){

			  }
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


/* ADC0_IRQn interrupt handler */
void ADC0_IRQHANDLER(void) {
  /*  Place your code here */
	float new_sample;
	if(ad8232_get_new_sample(&new_sample) == AD8232_SUCCESS){

	}
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F
     Store immediate overlapping exception return operation might vector to incorrect interrupt. */
  #if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
  #endif
}


