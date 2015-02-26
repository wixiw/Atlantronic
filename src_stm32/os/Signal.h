/*
 * Signal.h
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "os/queue.h"

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
class Signal
{
	public:
		Signal();

		void wait();
		void set();
		void setFromIsr();

	private:
		xQueueHandle queue;
};

//C fake struct
#else
	typedef struct Signal Signal;
#endif


#ifdef __cplusplus
extern "C" {
#endif

void set_signal(Signal* s);
void set_signal_FromIsr(Signal* s);
void wait_signal(Signal* s);

#ifdef __cplusplus
}
#endif

#endif /* SIGNAL_H_ */
