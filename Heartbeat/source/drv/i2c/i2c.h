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

/**
 * @brief Initializes I2C
 * @param dev : I2C module used for communication
 */

bool i2c_init(i2c_handle_t *dev);

/**
 * @brief Read data from I2C device
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param len : Length of data to read
 * @param datap : pointer to out buffer.
 *
 * @return *true* if successful.
 */
bool i2c_read(i2c_handle_t *dev, uint8_t devAddress, size_t len, uint8_t *datap);

/**
 * @brief Write data to I2C device
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param len : Length of data to write
 * @param datap : pointer to data to write.
 *
 * @return *true* if successful.
 */
bool i2c_write(i2c_handle_t *dev, uint8_t devAddress, size_t len, const uint8_t *datap);

/**
 * @brief Read data from subaddress
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param len : Length of data to read
 * @param datap : pointer to out buffer.
 *
 * @return *true* if successful.d
 */
bool i2c_read_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, size_t len, uint8_t *datap);

/**
 * @brief Write data to subaddress
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param len : Length of data to write
 * @param datap : pointer to data to write.
 *
 * @return *true* if successful.
 */
bool i2c_write_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, size_t len, const uint8_t* datap);

/**
 * @brief Read byte from subaddress
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param datap : pointer to out variable.
 *
 * @return *true* if successful.
 */
bool i2c_read_byte_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t *datap);

/**
 * @brief Write byte to subaddress
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param datap : byte to write
 *
 * @return *true* if successful.
 */
bool i2c_write_byte_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t data);

/**
 * @brief Read byte from subaddress, apply mask to read
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param mask : Mask
 * @param datap : pointer to out variable.
 *
 * @return *true* if successful.
 */
bool i2c_read_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t *datap);

/**
 * @brief Write byte to subaddress, apply mask to write (won't modify other bits' values)
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param mask : Mask
 * @param datap : byte to write (mask will be considered)
 *
 * @return *true* if successful.
 */
bool i2c_write_byte_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t mask, uint8_t data);

/**
 * @brief Read bits from subaddress, specify position of data and length in bits
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param start_bit : LSBit position of data
 * @param bit_length : number of bits in data
 * @param datap : pointer to out variable.
 *
 * @return *true* if successful.
 */
bool i2c_read_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t *datap);

/**
 * @brief Write bits to subaddress, specify position of data and length in bits. Other bits wont be modified
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param start_bit : LSBit position of data
 * @param bit_length : number of bits in data
 * @param datap : byte containing data to write
 *
 * @return *true* if successful.
 */
bool i2c_write_bits_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint8_t start_bit, uint8_t bit_length, uint8_t data);

/**
 * @brief Read word from subaddress
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param datap: pointer to out variable.
 *
 * @return *true* if successful.
 */
bool i2c_read_word_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint16_t *datap);

/**
 * @brief Write word to subaddress
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param datap : word to write
 *
 * @return *true* if successful.
 */
bool i2c_write_word_addr8(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint16_t data);

/**
 * @brief Read word from subaddress, apply mask to read
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param mask : Mask
 * @param datap : pointer to out variable.
 *
 * @return *true* if successful.
 */
bool i2c_read_word_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint16_t mask, uint16_t *datap);

/**
 * @brief Write word to subaddress, apply mask to write (won't modify other bits' values)
 * @param dev : I2C module used for communication
 * @param devAddress : I2C device address for communication
 * @param subAddr : I2C device subaddress to use
 * @param mask : Mask
 * @param datap : word to write (mask will be considered)
 *
 * @return *true* if successful.
 */
bool i2c_write_word_addr8_mask(i2c_handle_t *dev, uint8_t devAddress, uint8_t subAddr, uint16_t mask, uint16_t data);



#endif /* DRV_I2C_H_ */
