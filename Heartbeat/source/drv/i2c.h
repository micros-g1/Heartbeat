/*
 * i2c.h
 *
 *  Created on: 5 Nov 2020
 *      Author: grein
 */

#ifndef DRV_I2C_H_
#define DRV_I2C_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_i2c_freertos.h"

typedef i2c_rtos_handle_t i2c_handle_t;

//TODO: Documentation
bool i2c_init(i2c_handle_t *dev);
bool i2c_read(i2c_handle_t *dev, uint8_t devAddress, size_t len, uint8_t *datap);
bool i2c_write(i2c_handle_t *dev, uint8_t devAddress, size_t len, const uint8_t *datap);
bool i2c_read_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, size_t len, uint8_t *datap);
bool i2c_write_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, size_t len, const uint8_t* datap);
bool i2c_read_byte_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t *datap);
bool i2c_write_byte_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t data);
/**
 * @brief I2C Read Byte from 8-Bit SubAddress w/Mask
 * @details reads data from a device's 8-bit addressable register/subaddress.
 * This function is equivalent to performing a reading operation on the register
 * and then applying the given mask to the data that was just read.
 * @param dev : I2C device/module used to establish the communication.
 * @param devAddress : Address of the device to be read from.
 * @param subAddr : register/subaddress of the device to be read from.
 * @param mask : mask to be applied to the read data.
 * @param data : data that was read from the register and to which the mask was applied to.
 * @return *true* if read operation was successful. *false* otherwise.
 */
bool i2c_read_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t *datap);
/**
 * @brief I2C Write Byte to 8-Bit SubAddress w/Mask
 * @details writes data to a device's 8-bit addressable register/subaddress.
 * This function should be called when some bits of the register have to be left unchanged.
 * Hence, this function will involve a read operation to get the register's current information
 * and then a write operation to update its content.
 * @param dev : I2C device/module used to establish the communication.
 * @param devAddress : Address of the device to be written to.
 * @param subAddr : register/subaddress of the device to be written to.
 * @param mask : The set bits on mask will be the bits of data used to update the register's value.
 * The rest of the bits of data will be ignored.
 * @param data : data to be written on the register. The bits of data that are not set on mask will be ignored.
 * @return *true* if write operation was successful. *false* otherwise.
 */
bool i2c_write_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t data);
bool i2c_read_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t *datap);
bool i2c_write_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t data);

#endif /* DRV_I2C_H_ */
