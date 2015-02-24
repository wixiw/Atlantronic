/*
 * circular_buffer.c
 *
 *  Created on: Feb 20, 2015
 *      Author: willy
 */

#include "circular_buffer.h"
#include <string.h>

//return true if you can read sizeToRead until the end of the buffer
bool circular_canIReadContigousData(CircularBuffer const * const circularBuffer, size_t sizeToRead)
{
	unsigned int endOfRead = circularBuffer->start + sizeToRead;

	//detection d'overflow dans le int :
	if( endOfRead < circularBuffer->start || endOfRead < sizeToRead )
	{
		return false;
	}


	return endOfRead < circularBuffer->size ? true : false;
}

//return true if you can write sizeToRead until the end of the buffer
bool circular_canIWriteContigousData(CircularBuffer const * const circularBuffer, size_t sizeToWrite)
{
	unsigned int endOfWrite = circularBuffer->end + sizeToWrite;

	//detection d'overflow dans le int :
	if( endOfWrite < circularBuffer->end || endOfWrite < sizeToWrite )
	{
		return false;
	}


	return endOfWrite < circularBuffer->size ? true : false;
}

void circular_create(CircularBuffer * const circularBuffer, uint8_t * const linearBuffer, size_t const size)
{
	circularBuffer->data = linearBuffer;
	circularBuffer->size = size;
}

bool circular_pop(uint8_t* const linearBuffer, CircularBuffer * const circularBuffer, size_t const sizeToCopy)
{
	if( circular_getOccupiedRoom(circularBuffer) < sizeToCopy )
	{
		return false;
	}

	if( circular_canIReadContigousData(circularBuffer, sizeToCopy) )
	{
		// message deja contigu en memoire
		memcpy(linearBuffer, circularBuffer->data + circularBuffer->start, sizeToCopy);
	}
	else
	{
		size_t roomToEnd = circularBuffer->size - circularBuffer->start;
		//une partie est mise a la fin
		memcpy(linearBuffer, circularBuffer->data + circularBuffer->start, roomToEnd);
		//la suite au debut
		memcpy(linearBuffer + roomToEnd, circularBuffer->data, sizeToCopy - roomToEnd);
	}

	circularBuffer->start = (circularBuffer->start + sizeToCopy) % circularBuffer-> size;
	return true;
}

bool circular_append(CircularBuffer * const circularBuffer, uint8_t const * const linearBuffer, size_t const sizeToCopy)
{
	if( circular_getFreeRoom(circularBuffer) < sizeToCopy )
	{
		return false;
	}


	if( circular_canIWriteContigousData(circularBuffer, sizeToCopy) )
	{
		// message deja contigu en memoire
		memcpy(circularBuffer->data + circularBuffer->end, linearBuffer, sizeToCopy);
	}
	else
	{
		size_t roomToEnd = circularBuffer->size - circularBuffer->end;
		//une partie est mise a la fin
		memcpy(circularBuffer->data + circularBuffer->end, linearBuffer, roomToEnd);
		//la suite au debut
		memcpy(circularBuffer->data, linearBuffer + roomToEnd, sizeToCopy - roomToEnd);
	}

	circularBuffer->end = (circularBuffer->end + sizeToCopy) % circularBuffer-> size;
	return true;
}


size_t circular_getOccupiedRoom(CircularBuffer const * const circularBuffer)
{
	return (circularBuffer->end - circularBuffer->start) % circularBuffer->size;
}

size_t circular_getFreeRoom(CircularBuffer const * const circularBuffer)
{
	return circularBuffer->size - circular_getOccupiedRoom(circularBuffer);
}

void circular_reset(CircularBuffer * const circularBuffer)
{
	circularBuffer->start = 0;
	circularBuffer->end = 0;
	memset(circularBuffer->data, 0, circularBuffer->size);
}


