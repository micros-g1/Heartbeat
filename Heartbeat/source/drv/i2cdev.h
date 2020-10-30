/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * i2cdev.h - Functions to write to I2C devices
 */
/*
 ===============================================
 	 Modified:  19 Oct 2020
 	 Gonzalo Reina K.
 	 Just a few changes:
 	    * Removed all hardware dependent defines
 	    * fsl_i2c_freertos,  I2C_Dev type changed to i2c_handle_t
 	 Taken from: https://subak.io/code/crazyflie-firmware/crazyflie/src/drivers/interface/i2cdev.h.html
 ===============================================
 */


#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <stdint.h>
#include <stdbool.h>

#define I2CDEV_NO_MEM_ADDR  0xFF

#include "fsl_i2c_freertos.h"

typedef i2c_rtos_handle_t i2c_handle_t;


/**
 * Read bytes from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
               uint16_t len, uint8_t *data);

/**
 * Read bytes from an I2C peripheral with a 16bit internal reg/mem address
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevRead16(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
               uint16_t len, uint8_t *data);

/**
 * I2C device init function.
 * @param I2Cx  Pointer to I2C peripheral to initialize.
 *
 * @return TRUE if initialization went OK otherwise FALSE.
 */
int i2cdevInit(i2c_handle_t *dev);

/**
 * Read a byte from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadByte(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t *data);

/**
 * Read a bit from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number 0 - 7 to read.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBit(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitNum, uint8_t *data);
/**
 * Read up to 8 bits from an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to read from
 * @param devAddress  The device address to read from
 * @param memAddress  The internal address to read from, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to read, 1 - 8.
 * @param data  Pointer to a buffer to read the data to.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevReadBits(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data);

/**
 * Write bytes to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                 uint16_t len, uint8_t *data);

/**
 * Write bytes to an I2C peripheral with 16bit internal reg/mem address.
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param len  Number of bytes to read.
 * @param data  Pointer to a buffer to read the data from that will be written.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWrite16(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                   uint16_t len, uint8_t *data);

/**
 * Write a byte to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write from, I2CDEV_NO_MEM_ADDR if none.
 * @param data  The byte to write.
 *
 * @return TRUE if write was successful, otherwise FALSE.
 */
bool i2cdevWriteByte(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t data);

/**
 * Write a bit to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitNum  The bit number, 0 - 7, to write.
 * @param data  The bit to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBit(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data);

/**
 * Write up to 8 bits to an I2C peripheral
 * @param I2Cx  Pointer to I2C peripheral to write to
 * @param devAddress  The device address to write to
 * @param memAddress  The internal address to write to, I2CDEV_NO_MEM_ADDR if none.
 * @param bitStart The bit to start from, 0 - 7.
 * @param length  The number of bits to write, 1 - 8.
 * @param data  The byte containing the bits to write.
 *
 * @return TRUE if read was successful, otherwise FALSE.
 */
bool i2cdevWriteBits(i2c_handle_t *dev, uint8_t devAddress, uint8_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data);

#endif //__I2CDEV_H__
