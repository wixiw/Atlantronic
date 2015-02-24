/*
 * Signal.cxx
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#include "Signal.h"

Signal::Signal()
{
    queue = xQueueCreate(1,0);
}

void Signal::wait()
{
    xQueuePeek(queue, NULL, portMAX_DELAY);
}

void Signal::set()
{
    xQueueSend(queue, NULL, 0);
}

void Signal::setFromIsr()
{
    portBASE_TYPE xHigherPriorityTaskWoken = 0;
    xQueueSendFromISR(queue, NULL, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


/**
 * C wrappers
 */

void set_signal(Signal* s)
{
	s->set();
}

void wait_signal(Signal* s)
{
	s->wait();
}

