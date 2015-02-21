/*
 * circular_buffer.h
 *
 *  Created on: Feb 20, 2015
 *      Author: willy
 */

#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	uint8_t* data; //pointer on buffer start
	unsigned int size; //total size of the buffer
	unsigned int start; //pointer on data begin
	unsigned int end;   //pointer on data end +1 = next place to write

} CircularBuffer;

//initialize the circular buffer with a dedicated memory
//shall be called before any other call
void circular_create(CircularBuffer * const circularBuffer, uint8_t * const linearBuffer, size_t const size);

//pop (with copy) a circular buffer into a linear buffer
//@return false if the size doesn't match
bool circular_pop(uint8_t* const linearBuffer, CircularBuffer* const circularBuffer, size_t const sizeToCopy);

//append (with copy) a linear buffer at the end of a circular buffer
//@return false if the size doesn't match
bool circular_append(CircularBuffer * const circularBuffer, uint8_t const * const linearBuffer, size_t const sizeToCopy);

//return true if the circular buffer has not reached its end, so data are contigous in memory
bool circular_isDataContigous(CircularBuffer const * const circularBuffer);

//returns the occupied room in the circular buffer
size_t circular_getOccupiedRoom(CircularBuffer const * const circularBuffer);

//returns the free room in the circular buffer
size_t circular_getFreeRoom(CircularBuffer const * const circularBuffer);

//returns the occupied room to the end circular buffer
size_t circular_getOccupiedRoomToTheEnd(CircularBuffer const * const circularBuffer);

//returns the room to the end circular buffer
size_t circular_getEmptyRoomToTheEnd(CircularBuffer const * const circularBuffer);

#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_BUFFER_H_ */
