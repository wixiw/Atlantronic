/*
 * circular_buffer.c
 *
 *  Created on: Feb 20, 2015
 *      Author: willy
 */

#include "circular_buffer.h"
#include <string.h>

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

	if( circular_isDataContigous(circularBuffer) )
	{
		// message deja contigu en memoire
		memcpy(linearBuffer, circularBuffer->data + circularBuffer->start, sizeToCopy);
	}
	else
	{
		size_t sizeToTheEnd = circular_getOccupiedRoomToTheEnd(circularBuffer);
		//une partie est mise a la fin
		memcpy(linearBuffer, circularBuffer->data + circularBuffer->start, sizeToTheEnd);
		//la suite au debut
		memcpy(linearBuffer + sizeToTheEnd, circularBuffer->data, sizeToCopy - sizeToTheEnd);
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

	size_t roomToTheEnd = circular_getEmptyRoomToTheEnd(circularBuffer);
	if( sizeToCopy <= roomToTheEnd)
	{
		// message deja contigu en memoire
		memcpy(circularBuffer->data + circularBuffer->end, linearBuffer, sizeToCopy);
	}
	else
	{
		//une partie est mise a la fin
		memcpy(circularBuffer->data + circularBuffer->end, linearBuffer, roomToTheEnd);
		//la suite au debut
		memcpy(circularBuffer->data, linearBuffer + roomToTheEnd, sizeToCopy - roomToTheEnd);
	}

	circularBuffer->end = (circularBuffer->end + sizeToCopy) % circularBuffer-> size;
	return true;
}

bool circular_isDataContigous(CircularBuffer const * const circularBuffer)
{
	return circularBuffer->start < circularBuffer->end ? true : false;
}

size_t circular_getOccupiedRoom(CircularBuffer const * const circularBuffer)
{
	return (circularBuffer->end - circularBuffer->start) % circularBuffer->size;
}

size_t circular_getFreeRoom(CircularBuffer const * const circularBuffer)
{
	return circularBuffer->size - circular_getOccupiedRoom(circularBuffer);
}

size_t circular_getOccupiedRoomToTheEnd(CircularBuffer const * const circularBuffer)
{
	if( circular_isDataContigous(circularBuffer) )
	{
		return circularBuffer->end - circularBuffer->start;
	}
	else
	{
		return circularBuffer->size - circularBuffer->start;
	}
}

size_t circular_getEmptyRoomToTheEnd(CircularBuffer const * const circularBuffer)
{
	return circularBuffer->size - circularBuffer->end;
}


