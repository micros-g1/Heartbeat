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

/*----------------------------------------------------------------
 * MP3 Wrapper
 * Wrapper of MP3 function, simplifies interface for decoding MP3
 *---------------------------------------------------------------*/

/*!
 * @brief Initialises the MP3Wrap.
 *
 * This function initialises the MP3Wrap.
 * The default configuration is set to the following values.
 *
 * @return True if initialised successfully.
 */
bool mp3wrap_init();

/*!
 * @brief Sets pointer to data to decode
 *
 * This function sets the MP3 data that will be decoded
 *
 * @param data : pointer to MP3 data
 * @param datalength : length in bytes of mp3 data
 *
 * @return True if MP3 frame could be found within data.
 */
bool mp3wrap_setdata(const uint8_t* data, size_t datalength);

/*!
 * @brief Decodes next frame of MP3 data
 *
 * This function decodes the next frame of MP3 data. The format of the output data will correspond
 * to the format of the MP3 data (stereo data will be decompressed as LRLR bytes)
 *
 * @param outdata : pointer to output buffer, with enough space for one MP3 frame
 * @param outbytes : bytes decoded (number of bytes in outdata). Zero in case the data has been completely decoded.
 *
 * @return False if problem while decoding data.
 */
bool mp3wrap_decode_next(uint8_t* outdata, size_t* outbytes);

/*!
 * @brief Indicates if data has been fully decoded
 *
 * Indicates when MP3 data has been fully decoded. Can be used to detect end of data.
 *
 * @return True if decoding is finished (or if no decoding in course)
 */
bool mp3wrap_finished();

/*!
 * @brief Initialises the MP3Wrap.
 *
 * Deinitialises the MP3Wrap
 */
void mp3wrap_deinit();


#endif /* DRV_MP3WRAP_H_ */
