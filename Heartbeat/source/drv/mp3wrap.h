/*
 * mp3wrap.h
 *
 *  Created on: 4 Feb 2021
 *      Author: grein
 */

#ifndef DRV_MP3WRAP_H_
#define DRV_MP3WRAP_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

bool mp3wrap_init();
bool mp3wrap_setdata(const uint8_t* data, size_t datalength);
bool mp3wrap_decode_next(uint8_t* outdata, size_t* outbytes);
bool mp3wrap_finished();
void mp3wrap_deinit();


#endif /* DRV_MP3WRAP_H_ */
