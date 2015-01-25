/*
 * signal.h
 *
 *  Created on: Jan 25, 2015
 *      Author: willy
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_

#include "queue.h"

class signal
{
    public:
        signal();
        virtual ~signal();

        void wait();
        void set();
        void setFromIsr();

    private:
        xQueueHandle queue;
};

#endif /* SIGNAL_H_ */
