/*
 * orders.h
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef ORDERS_H_
#define ORDERS_H_

#include <boost/shared_ptr.hpp>

#include "ModeSelector.hpp"
#include "MotionOrder.hpp"
#include "OmnidirectOrder2.hpp"
#include "OpenloopOrder.hpp"
#include "ReplayOrder.hpp"

using namespace boost;

namespace arp_ods
{
    namespace orders
    {
        extern shared_ptr<MotionOrder> defaultOrder;
    }

}

#endif /* ORDERS_H_ */
