/*
 * Copyright 2019-2020 NXP.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

/*! @name PORTC3 (number 73), IRQ
  @{ */
#define BOARD_INITGT202SHIELD_IRQ_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITGT202SHIELD_IRQ_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITGT202SHIELD_IRQ_PIN 3U     /*!<@brief PORTC pin index: 3 */
                                             /* @} */

/*! @name PORTC12 (number 84), PWRON
  @{ */
#define BOARD_INITGT202SHIELD_PWRON_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITGT202SHIELD_PWRON_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITGT202SHIELD_PWRON_PIN 12U    /*!<@brief PORTC pin index: 12 */
                                               /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitGT202Shield(void);

/*! @name PORTB9 (number 57), IRQ
  @{ */
#define BOARD_INITWIFI10CLICKSHIELD_IRQ_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITWIFI10CLICKSHIELD_IRQ_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITWIFI10CLICKSHIELD_IRQ_PIN 9U     /*!<@brief PORTB pin index: 9 */
                                                   /* @} */

/*! @name PORTC2 (number 72), PWRON
  @{ */
#define BOARD_INITWIFI10CLICKSHIELD_PWRON_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITWIFI10CLICKSHIELD_PWRON_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITWIFI10CLICKSHIELD_PWRON_PIN 2U     /*!<@brief PORTC pin index: 2 */
                                                     /* @} */

/*! @name PORTB2 (number 55), KFET
  @{ */
#define BOARD_INITWIFI10CLICKSHIELD_KFET_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITWIFI10CLICKSHIELD_KFET_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITWIFI10CLICKSHIELD_KFET_PIN 2U     /*!<@brief PORTB pin index: 2 */
                                                    /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitWiFi10ClickShield(void);

/*! @name PORTB9 (number 57), IRQ
  @{ */
#define BOARD_INITSILEX2401SHIELD_IRQ_GPIO GPIOB /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITSILEX2401SHIELD_IRQ_PORT PORTB /*!<@brief PORT device name: PORTB */
#define BOARD_INITSILEX2401SHIELD_IRQ_PIN 9U     /*!<@brief PORTB pin index: 9 */
                                                 /* @} */

/*! @name PORTC2 (number 72), PWRON
  @{ */
#define BOARD_INITSILEX2401SHIELD_PWRON_GPIO GPIOC /*!<@brief GPIO device name: GPIOC */
#define BOARD_INITSILEX2401SHIELD_PWRON_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITSILEX2401SHIELD_PWRON_PIN 2U     /*!<@brief PORTC pin index: 2 */
                                                   /* @} */

/*! @name PORTA2 (number 36), KFET
  @{ */
#define BOARD_INITSILEX2401SHIELD_KFET_GPIO GPIOA /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITSILEX2401SHIELD_KFET_PORT PORTA /*!<@brief PORT device name: PORTA */
#define BOARD_INITSILEX2401SHIELD_KFET_PIN 2U     /*!<@brief PORTA pin index: 2 */
                                                  /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitSilex2401Shield(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
