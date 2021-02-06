/*
 * mp3wrap.c
 *
 *  Created on: 4 Feb 2021
 *      Author: grein
 */
#include "mp3wrap.h"
#include "libs/helix/pub/mp3dec.h"

HMP3Decoder dec = 0;
uint8_t* datap = 0;
int bytesleft = 0;

//TODO: ERROR WHEN DATA DOES NOT MEET THIS CRITERIA
#define TARGET_SAMPLERATE 16000
#define TARGET_CHANNELS 1
#define TARGET_BITS 16


bool mp3wrap_init()
{
	dec = MP3InitDecoder();
	datap = NULL;
	bytesleft = 0;
	return dec != 0; // If NULL memory allocation failed.
}

bool mp3wrap_setdata(const uint8_t* data, size_t datalength)
{
	bool framefound = false;
	datap = (uint8_t*) data;
	bytesleft = datalength;
	while(!framefound && bytesleft > 0)
	{
		int offset = MP3FindSyncWord(datap,  bytesleft);
		if(offset < 0)
			break; //Sync word not found, or error.
		datap+= offset;
		bytesleft -= offset;
		MP3FrameInfo frameInfo;
		int ercode = MP3GetNextFrameInfo(dec, &frameInfo, datap);
		if(ercode == ERR_MP3_NONE)
		{
			framefound = true;
		}
		else
		{
			datap++;
			bytesleft--;
		}
	}
	if(!framefound)
	{
		datap = NULL;
		bytesleft = 0;
	}
	return framefound;
}

bool mp3wrap_decode_next(uint8_t* outdata, size_t* outbytes)
{
	bool success = true;
	*outbytes = 0;
	if(bytesleft != 0)
	{
		if(MP3Decode(dec, &datap, &bytesleft, (short*) outdata, 0) != ERR_MP3_NONE)
			success = false;
		else
		{
			MP3FrameInfo mp3FrameInfo;
			MP3GetLastFrameInfo(dec, &mp3FrameInfo);
			*outbytes = mp3FrameInfo.outputSamps*mp3FrameInfo.bitsPerSample/8;
		}
	}
	return success;
}

bool mp3wrap_finished()
{
	return bytesleft == 0;
}

void mp3wrap_deinit()
{
	MP3FreeDecoder(dec);
	datap = NULL;
	bytesleft = 0;
}
