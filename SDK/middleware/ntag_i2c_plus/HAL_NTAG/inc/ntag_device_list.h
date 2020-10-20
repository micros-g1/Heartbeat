/*
 * Copyright 2016 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
/* DO NOT USE INCLUDE GUARDS - BREAKS LIST GENERATION */

/*                LABEL  I2C-ADDRESS  ISR */
#if defined(FRDM_K82F)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTC)
#endif
#if defined(FRDM_K64F)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTA)
#endif
#if defined(HEXIWEAR)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTB)
#endif
#if defined(FRDM_KL27Z)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTE)
#endif
#if defined(FRDM_KL43Z)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTA)
#endif
#if defined(FRDM_K32L2B)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTA)
#endif
#if defined(FRDM_K22F)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTB)
#endif
#if defined(FRDM_KL02Z)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTA)
#endif
#if defined(FRDM_KW41Z)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_PORTC)
#endif
#if defined(CPU_LPC55S69JBD100_cm33_core0)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_IOCON)
#endif
#if defined(CPU_LPC55S28JBD100)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_IOCON)
#endif
#if defined(CPU_LPC55S16JBD100)
NTAG_DEVICE_ENTRY(NTAG0, 0x55, ISR_INT_IOCON)
#endif

/* DO NOT USE INCLUDE GUARDS - BREAKS LIST GENERATION */
