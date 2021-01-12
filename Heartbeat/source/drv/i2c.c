/*
 * i2c.c
 *
 *  Created on: 5 Nov 2020
 *      Author: grein
 */
#include "i2c.h"
#include "fsl_i2c_freertos.h"
#include "peripherals.h"

bool i2c_init(i2c_handle_t *dev)
{
	// I2C initialised by hardware initialisation
	// Just return true.
	return true;
}

bool i2c_read(i2c_handle_t *dev, uint8_t devAddress, size_t len, uint8_t *data)
{
	//I2C Transfer
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	//Address
	xfer.slaveAddress   = devAddress;
	//Read
	xfer.direction      = kI2C_Read;
	xfer.data           = data;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	//No subAddr
	xfer.subaddress     = 0;
	xfer.subaddressSize = 0;
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2c_write(i2c_handle_t *dev, uint8_t devAddress, size_t len, const uint8_t *data)
{
	//I2C Transfer
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	//Address
	xfer.slaveAddress   = devAddress;
	//Write
	xfer.direction      = kI2C_Write;
	xfer.data           = (uint8_t*) data;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	//No subAddr
	xfer.subaddress     = 0;
	xfer.subaddressSize = 0;
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2c_read_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, size_t len, uint8_t *datap)
{
	//I2C Transfer
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.slaveAddress   = devAddress;
	//Read
	xfer.direction      = kI2C_Read;
	xfer.data           = datap;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	//8 Bits subAddr
	xfer.subaddress     = subAddr;
	xfer.subaddressSize = 1;
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2c_write_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, size_t len, const uint8_t* datap)
{
	//I2C Transfer
	i2c_master_transfer_t xfer;
	memset(&xfer, 0, sizeof(xfer));
	xfer.slaveAddress   = devAddress;
	//Write
	xfer.direction      = kI2C_Write;
	xfer.data           = (uint8_t*) datap;
	xfer.dataSize       = len;
	xfer.flags          = kI2C_TransferDefaultFlag;
	//8 Bits subAddr
	xfer.subaddress     = subAddr;
	xfer.subaddressSize = 1;
	return I2C_RTOS_Transfer(dev, &xfer) == kStatus_Success;
}

bool i2c_read_byte_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t *datap)
{
	//1 byte I2C Read
	return i2c_read_addr8(dev, devAddress, subAddr, 1, datap);
}

bool i2c_write_byte_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t data)
{
	//1 byte I2C write
	return i2c_write_addr8(dev, devAddress, subAddr, 1, &data);
}

bool i2c_read_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t *datap)
{
	//Read byte
	bool success = i2c_read_byte_addr8(dev, devAddress, subAddr, datap);
	if(success)
		//Apply read mask
		*datap = *datap & mask;
	return success;
}

bool i2c_write_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t data)
{
	uint8_t newdata = 0;
	bool success = true;
	//Any bit to be conserved?
	if(~mask)
		//If so, perform a reading operation, clearing those bits that should be ignored by what's indicated on mask.
		success = i2c_read_byte_addr8_mask(dev, devAddress, subAddr, ~mask, &newdata);
	if(success)
	{
		//Apply mask to incoming data, and or it with old data
		newdata |= (~mask & new_data)(data & mask);
		//write
		success = i2c_write_byte_addr8(dev, devAddress, subAddr, newdata);
	}
	return success;
}

bool i2c_read_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t *datap)
{
	//Generate mask for bits
	uint8_t mask = ((1 << bit_length) - 1) << (start_bit + 1 - bit_length);
	//perform read with mask
	bool success = i2c_read_byte_addr8_mask(dev, devAddress, subAddr, mask, datap);
	if(success)
	{
		//Shift data
		*datap = *datap >> (start_bit + 1 - bit_length);
	}
	return success;
}

bool i2c_write_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t data)
{
	//Generate mask for bits
	uint8_t mask = ((1 << bit_length) - 1) << (start_bit + 1 - bit_length);
	//Shift data
	data = data << (start_bit + 1 - bit_length);
	//perform write with mask
	return i2c_write_byte_addr8_mask(dev, devAddress, subAddr, mask, data);
}
