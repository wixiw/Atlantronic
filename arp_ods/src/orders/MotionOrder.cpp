/*
 * MotionOrder.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "math/math.hpp"
#include "MotionOrder.hpp"
#include "orders/orders.h"

using namespace arp_core;
using namespace arp_math;

namespace arp_ods
{

namespace order
{
    shared_ptr < MotionOrder > defaultOrder(new MotionOrder());
}

//MotionOrder::MotionOrder(const MotionOrder& order) :
//    ModeSelector(static_cast<ModeSelector&>(order) )
//{
//    m_type = order.m_type;
//    m_reverse = order.m_reverse;
//    //m_id != m_id !!
//}

MotionOrder::MotionOrder() :
    arp_ods::ModeSelector(), m_type(NO_ORDER), m_reverse(false)
{

}

Velocity MotionOrder::computeSpeed(arp_core::Pose currentPosition)
{
    Velocity v;
    v.linear = 0;
    v.angular = 0;
    return v;
}

Pose MotionOrder::reversePosition(Pose p)
{
    Pose ret;
    ret.x = p.x;
    ret.y = p.y;
    if( m_reverse )
    {
        ret.theta = normalizeAngle(p.theta + PI);
    }
    else
    {
        ret.theta = normalizeAngle(p.theta);
    }
    return ret;
}

OrderType MotionOrder::getType() const
{
    return m_type;
}

std::string MotionOrder::getTypeString() const
{
    switch (m_type)
    {
        case NO_ORDER:
            return "NO_ORDER";
            break;
        case STAY_IN_POSITION:
            return "STAY_IN_POSITION";
            break;
        case TRANSLATE:
            return "TRANSLATE";
            break;
        case ROTATE:
            return "ROTATE";
            break;
        case FANTOM:
            return "FANTOM";
            break;
        default:
            return "ORDER_UNKNOW";
            break;
    }
}

void MotionOrder::setReverse(bool reverse)
{
    m_reverse = reverse;
}

void MotionOrder::setId(int id)
{
    m_id = id;
}

}

