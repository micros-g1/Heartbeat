/*
 * uda1380.c
 *
 *  Created on: 31 Jan 2021
 *      Author: grein
 */

#include "uda1380_hw.h"
#include "uda1380.h"
#include <peripherals.h>
#include "i2c/i2c.h"

static i2c_handle_t* i2c = NULL;
static sai_transfer_t xfer = {0};
static uda1380_callback_t cb = NULL;

static void callback(I2S_Type *base, sai_handle_t *handle, status_t status, void *userData)
{
	//Execute external callback if one was set.
	if(cb) cb();
}

bool uda1380_init()
{
	bool success = true;
	i2c = &I2CA_rtosHandle;
	if((success = i2c_init(i2c)))
	{
		//Reset UDA1380
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_RESET_ADDR, 0x0000, 0x0000) : false;
		//Send config values according to defined configuration
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_CLK_ADDR, UDA1380_CLK_INIT_MASK, UDA1380_CLK_INIT_VALUE) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_IBUSCFG_ADDR, UDA1380_IBUSCFG_INIT_MASK, UDA1380_IBUSCFG_INIT_VALUE) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_PWRCTR_ADDR, UDA1380_PWRCTR_INIT_MASK, UDA1380_PWRCTR_INIT_VALUE) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_AMIXCFG_ADDR, UDA1380_AMIXCFG_INIT_MASK, UDA1380_AMIXCFG_INIT_VALUE) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_HPHAMPCFG_ADDR, UDA1380_HPHAMPCFG_INIT_MASK, UDA1380_HPHAMPCFG_INIT_MASK) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_MVOLCTR_ADDR, UDA1380_MVOLCTR_INIT_MASK, UDA1380_MVOLCTR_INIT_VALUE) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_MIXVOLCTR_ADDR, UDA1380_MIXVOLCTR_INIT_MASK, UDA1380_MIXVOLCTR_INIT_VALUE) : false;
		success = success ? i2c_write_word_addr8_mask(i2c,UDA1380_I2C_ADDR, UDA1380_DEEMP_ADDR, UDA1380_DEEMP_INIT_MASK, UDA1380_DEEMP_INIT_VALUE) : false;
		//Set I2S driver callback
		I2S0_Tx_handle.callback = callback;
	}
	return success;
}

void uda1380_playback(const uint8_t* data, size_t data_length)
{
	uint32_t temp = (uint32_t)data;
	xfer.data = (uint8_t *)temp;
	xfer.dataSize = data_length;
	//Play chunk of data
	SAI_TransferSendNonBlocking(I2S0_PERIPHERAL, &I2S0_Tx_handle, &xfer);
}

void uda1380_stop()
{
	//Stop playing data
	SAI_TransferAbortSend(I2S0_PERIPHERAL, &I2S0_Tx_handle);
}

void uda1380_finished_set_callback(uda1380_callback_t callback)
{
	//Set callback
	cb = callback;
}

