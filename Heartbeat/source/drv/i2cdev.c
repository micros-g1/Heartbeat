//This file is based on file:
// https://subak.io/code/crazyflie-firmware/crazyflie/src/drivers/src/i2cdev_f405.c.html

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
 * i2cdev.c - Functions to write to I2C devices
 */

/*
 ===============================================
 	 Modified:  19 Oct 2020
 	 Gonzalo Reina K.
 	 (... including without limitation the rights  to use, copy, modify ...)
 	 * Functions that were not hardware dependent are used without modifications
 	 * Hardware dependent functions modified in order to use I2C_RTOS Driver
 ===============================================
 */


#include "i2cdev.h"
#include "fsl_i2c_freertos.h"
#include "peripherals.h"

int i2cdevInit(i2c_handle_t *dev)
{
	// I2C initialised by hardware initialisation
	// Just return true.
	return true;
}

bool i2cdevReadByte(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                    uint8_t *data)
{
	return i2cdevRead(dev, devAddress, memAddress,1,data);
}

bool i2cdevReadBit(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                     uint8_t bitNum, uint8_t *data)
{
	uint8_t byte;
	bool status;

	status = i2cdevRead(dev, devAddress, memAddress, 1, &byte);
	*data = byte & (1 << bitNum);

	return status;
}

bool i2cdevReadBits(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
	bool status;
	uint8_t byte;

	if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		byte &= mask;
		byte >>= (bitStart - length + 1);
		*data = byte;
	}
	return status;
}

bool i2cdevRead(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
               uint16_t len, uint8_t *data)
{
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.slaveAddress   = devAddress;
	xfer.direction      = kI2C_Read;
	xfer.data           = data;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	if(devAddress == I2CDEV_NO_MEM_ADDR)
	{
		xfer.subaddress     = 0;
		xfer.subaddressSize = 0;
	}
	else
	{
		xfer.subaddress     = (uint8_t) memAddress;
		xfer.subaddressSize = 1;
	}
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2cdevRead16(i2c_handle_t *dev, uint8_t devAddress, uint32_t memAddress,
               uint16_t len, uint8_t *data)
{
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.slaveAddress   = devAddress;
	xfer.direction      = kI2C_Read;
	xfer.data           = data;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	if(devAddress == I2CDEV_NO_MEM_ADDR)
	{
		xfer.subaddress     = 0;
		xfer.subaddressSize = 0;
	}
	else
	{
		xfer.subaddress     = (uint16_t) memAddress;
		xfer.subaddressSize = 2;
	}
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2cdevWriteByte(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                    uint8_t data)
{
	return i2cdevWrite(dev, devAddress, memAddress, 1, &data);
}

bool i2cdevWriteBit(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
	uint8_t byte;
	i2cdevReadByte(dev, devAddress, memAddress, &byte);
	byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
	return i2cdevWriteByte(dev, devAddress, memAddress, byte);
}

bool i2cdevWriteBits(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
	bool status;
	uint8_t byte;

	if ((status = i2cdevReadByte(dev, devAddress, memAddress, &byte)) == true)
	{
	  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	  data <<= (bitStart - length + 1); // shift data into correct position
	  data &= mask; // zero all non-important bits in data
	  byte &= ~(mask); // zero all important bits in existing byte
	  byte |= data; // combine data with existing byte
	  status = i2cdevWriteByte(dev, devAddress, memAddress, byte);
	}
	return status;
}

bool i2cdevWrite(i2c_handle_t *dev, uint8_t devAddress, uint16_t memAddress,
                uint16_t len, uint8_t *data)
{
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.slaveAddress   = devAddress;
	xfer.direction      = kI2C_Write;
	xfer.data           = data;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	if(devAddress == I2CDEV_NO_MEM_ADDR)
	{
		xfer.subaddress     = 0;
		xfer.subaddressSize = 0;
	}
	else
	{
		xfer.subaddress     = (uint8_t) memAddress;
		xfer.subaddressSize = 1;
	}
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2cdevWrite16(i2c_handle_t *dev, uint8_t devAddress, uint32_t memAddress,
                   uint16_t len, uint8_t *data)
{
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.slaveAddress   = devAddress;
	xfer.direction      = kI2C_Write;
	xfer.data           = data;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	if(devAddress == I2CDEV_NO_MEM_ADDR)
	{
		xfer.subaddress     = 0;
		xfer.subaddressSize = 0;
	}
	else
	{
		xfer.subaddress     = (uint16_t) memAddress;
		xfer.subaddressSize = 2;
	}
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}
