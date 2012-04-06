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
#include "StayInPosition.hpp"
#include "FantomOrder.hpp"
#include "RotationOrder.hpp"

using namespace boost;

namespace arp_ods
{
    namespace order
    {
        extern shared_ptr<MotionOrder> defaultOrder;
    }

}

#endif /* ORDERS_H_ */
