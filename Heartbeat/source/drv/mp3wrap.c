/*
 * mp3wrap.c
 *
 *  Created on: 4 Feb 2021
 *      Author: grein
 */
#include "mp3wrap.h"
#include "libs/helix/pub/mp3dec.h"

static HMP3Decoder dec = 0;
static uint8_t* datap = 0;
static int bytesleft = 0;

bool mp3wrap_init()
{
	dec = MP3InitDecoder();
	//Initialise variables
	datap = NULL;
	bytesleft = 0;
	return dec != 0; // If NULL memory allocation failed.
}

bool mp3wrap_setdata(const uint8_t* data, size_t datalength)
{
	//Try to find mp3 frame
	bool framefound = false;
	datap = (uint8_t*) data;
	bytesleft = datalength;
	if(data != NULL)
		while(!framefound && bytesleft > 0)
		{
			int offset = MP3FindSyncWord(datap,  bytesleft);
			if(offset < 0)
				break; //Sync word not found, or error.
			datap+= offset;
			bytesleft -= offset;
			MP3FrameInfo frameInfo;
			int ercode = MP3GetNextFrameInfo(dec, &frameInfo, datap);
			//Is mp3 frame valid?
			if(ercode == ERR_MP3_NONE)
			{
				framefound = true;
			}
			else
			{
				//If no, try next one.
				datap++;
				bytesleft--;
			}
		}
	if(!framefound)
	{
		//Could not find valid data?
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
		//Decode next data
		if(MP3Decode(dec, &datap, &bytesleft, (short*) outdata, 0) != ERR_MP3_NONE)
			success = false;
		else
		{
			//Number of decoded bytes
			MP3FrameInfo mp3FrameInfo;
			MP3GetLastFrameInfo(dec, &mp3FrameInfo);
			*outbytes = mp3FrameInfo.outputSamps*mp3FrameInfo.bitsPerSample/8;
		}
	}
	return success;
}

bool mp3wrap_finished()
{
	//when no bytes left to read, then finished.
	return bytesleft == 0;
}

void mp3wrap_deinit()
{
	//Deinitialise.
	MP3FreeDecoder(dec);
	datap = NULL;
	bytesleft = 0;
}
