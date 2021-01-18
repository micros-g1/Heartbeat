/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_adc16.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "fsl_uart.h"
#include "fsl_uart_freertos.h"
#include "fsl_clock.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Alias for ADC0 peripheral */
#define ADC0_PERIPHERAL ADC0
/* ADC0 interrupt vector ID (number). */
#define ADC0_IRQN ADC0_IRQn
/* ADC0 interrupt handler identifier. */
#define ADC0_IRQHANDLER ADC0_IRQHandler
/* Channel 0 (SE.2) conversion control group. */
#define ADC0_CH0_CONTROL_GROUP 0
/* Alias for GPIOB peripheral */
#define GPIOB_GPIO GPIOB
/* Alias for PORTB */
#define GPIOB_PORT PORTB
/* GPIOB interrupt vector ID (number). */
#define GPIOB_IRQN PORTB_IRQn
/* GPIOB interrupt handler identifier. */
#define GPIOB_IRQHANDLER PORTB_IRQHandler
/* BOARD_InitPeripherals defines for I2C0 */
/* Definition of peripheral ID */
#define I2C0_PERIPHERAL I2C0
/* Definition of the clock source */
#define I2C0_CLOCK_SOURCE I2C0_CLK_SRC
/* Definition of the clock source frequency */
#define I2C0_CLK_FREQ CLOCK_GetFreq(I2C0_CLOCK_SOURCE)
/* Definition of peripheral ID */
#define UART0_PERIPHERAL UART0
/* Definition of the clock source frequency */
#define UART0_CLOCK_SOURCE CLOCK_GetFreq(UART0_CLK_SRC)
/* Definition of the backround buffer size */
#define UART0_BACKGROUND_BUFFER_SIZE 100

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern adc16_channel_config_t ADC0_channelsConfig[1];
extern const adc16_config_t ADC0_config;
extern const adc16_channel_mux_mode_t ADC0_muxMode;
extern const adc16_hardware_average_mode_t ADC0_hardwareAverageMode;
extern i2c_rtos_handle_t I2CA_rtosHandle;
extern const i2c_master_config_t I2C0_config;
extern uart_rtos_handle_t UART0_rtos_handle;
extern uart_handle_t UART0_uart_handle;
extern uart_rtos_config_t UART0_rtos_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
