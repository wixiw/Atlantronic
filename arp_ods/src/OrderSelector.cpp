/*
 * OrderSelector.cpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#include "OrderSelector.hpp"

namespace arp_ods
{

OrderSelector::OrderSelector():
        m_requestedOrder(order::defaultOrder),
        m_currentOrder(order::defaultOrder),
        m_state(IDLE),
        m_interruptRequest(false),
        m_workTimeout(15),
        m_haltLinearVelocity(0.010),
        m_haltAngularVelocity(0.2)
{

}

void OrderSelector::switchState(arp_core::Pose currentPose)
{
    switch (m_state) {
        case IDLE:
            doIdle(currentPose);
            break;
        case WORK:
            doWork(currentPose);
            break;
        case HALT:
            doHalt(currentPose);
            break;
        default:
            break;
    }
}

void OrderSelector::doIdle(arp_core::Pose currentPose)
{
    //if a new order is present
    if( m_requestedOrder->getType() != NO_ORDER )
    {
        ROS_INFO("New order received : %s", m_requestedOrder->getTypeString().c_str()  );
        m_currentOrder = m_requestedOrder;
        m_requestedOrder = order::defaultOrder;
        m_state = WORK;
        m_orderBeginTime = ros::Time::now().toSec();
        return;
    }
}

void OrderSelector::doWork(arp_core::Pose currentPose)
{
    //We received an interrupt request, so we go into the HALT state who will stop the robot gently
    if( m_interruptRequest == true )
    {
        ROS_INFO("switch to HALT state from WORK because of an interruption");
        m_state = HALT;
        return;
    }

    //we have finished an order
    if( m_currentOrder->getMode() == MODE_DONE )
    {
        //nothing to do, go into HALT (we are maybe not stopped, but it will be the job of the default order)
        ROS_INFO("switch to HALT state from WORK because of the order is finished");
        m_state = HALT;
        return;
    }

    //TODO WLA : le timeout devrait appartenir à l'ordre, mais là j'ai la flemme
    //The robot did not succeed the order in a predifined time, the order is canceled
    double orderDuration = ros::Time::now().toSec() - m_orderBeginTime;
    if( orderDuration < 0 || orderDuration > m_workTimeout )
    {
        //TODO WLA passer l'ordre en erreur
        ROS_ERROR("switch to HALT state from WORK because of the order is too long to finished");
        m_state = HALT;
        return;
    }
}

void OrderSelector::doHalt(arp_core::Pose currentPose)
{
    //TODO WLA ajouter une condition sur le temps passé à l'arrêt

    if( currentPose.linear_velocity <= m_haltLinearVelocity
            && currentPose.linear_velocity <= m_haltAngularVelocity)
    {
        ROS_INFO("switch to IDLE state from HALT");
        m_state = HALT;
        return;
    }
}

void OrderSelector::interrupt()
{
    m_interruptRequest = true;
}

shared_ptr<MotionOrder> OrderSelector::getCurrentOrder()
{
    if( m_currentOrder == NULL )
    {
        ROS_ERROR("MotionOrder : m_currentOrder should not be null");
        m_currentOrder = order::defaultOrder;
    }

    return m_currentOrder;
}

shared_ptr<MotionOrder> OrderSelector::getRequestedOrder()
{
    if( m_requestedOrder == NULL )
    {
        ROS_ERROR("MotionOrder : m_requestedOrder should not be null");
        m_requestedOrder = order::defaultOrder;
    }

    return m_currentOrder;
}

MotionState OrderSelector::getMotionState() const
{
    return m_state;
}

void OrderSelector::setWorkTimeout(double timeout)
{
    if( timeout > 0 && timeout < 100 )
    {
        m_workTimeout = timeout;
    }
    else
    {
        ROS_WARN("OrderSelector did not set a new timeout because it is out of bounds");
    }
}

void OrderSelector::setHaltLinearVelocity(double maxSpeed)
{
    if( maxSpeed > 0 && maxSpeed < 1 )
    {
        m_haltLinearVelocity = maxSpeed;
    }
    else
    {
        ROS_WARN("OrderSelector did not set a new halt linear speed because it is out of bounds");
    }
}

void OrderSelector::setHaltAngularVelocity(double maxSpeed)
{
    if( maxSpeed > 0 && maxSpeed < 1 )
    {
        m_haltAngularVelocity = maxSpeed;
    }
    else
    {
        ROS_WARN("OrderSelector did not set a new halt angular speed because it is out of bounds");
    }
}

}


// Displaying:
std::ostream& operator<<(std::ostream& os, const arp_ods::MotionState& state)
{
    switch (state)
    {
        case arp_ods::IDLE:
            return os << "STATE_IDLE";
            break;
        case arp_ods::WORK:
            return os << "STATE_WORKING";
            break;
        case arp_ods::HALT:
            return os << "STATE_HALT";
            break;
        default:
            return os << "STATE_UNKNOWN";
            break;
    }
}

