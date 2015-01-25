/*
 * signal.cxx
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#include "signal.h"

signal::signal()
{
    queue = xQueueCreate(1,0);
}

signal::~signal()
{

}

void signal::wait()
{
    xQueuePeek(queue, NULL, portMAX_DELAY);
}

void signal::set()
{
    xQueueSend(gpio_queue_go, NULL, 0);
}

void signal::setFromIsr()
{
    portBASE_TYPE xHigherPriorityTaskWoken = 0;
    xQueueSendFromISR(gpio_queue_go, NULL, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
