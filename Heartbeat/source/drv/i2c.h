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
bool i2c_read_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t *datap);
bool i2c_write_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t data);
bool i2c_read_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t *datap);
bool i2c_write_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t data);

#endif /* DRV_I2C_H_ */
