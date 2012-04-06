/*
 * LittleSexControl.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "LittleSexControl.hpp"
#include <rtt/Component.hpp>

using namespace arp_ods;

ORO_LIST_COMPONENT_TYPE( arp_ods::LittleSexControl )

LittleSexControl::LittleSexControl(const std::string& name):
        OdsTaskContext(name),
        attrOrder(order::defaultOrder)
{
    createOrocosInterface();
}

void LittleSexControl::getInputs()
{
    if( inOrder.readNewest(attrOrder) == NewData )
    {
        LOG(Info) << "received a new order" << endlog();
    }

    //faut-il tester que c'est bien mis à jour ?
    inPosition.readNewest(attrPosition);
}

void LittleSexControl::updateHook()
{
    //bufferise inputs
    getInputs();

    //compute current order mode
    attrOrder->switchMode(attrPosition);

    // calcule les consignes
    //TODO recuperer le dt depuis le dernier appel = inClock - lastInClock
    double dt = 0.0;
    attrComputedTwistCmd = attrOrder->computeSpeed(attrPosition,dt);

    //check wheel blocked
    checkWheelBlocked();

    //publish computed value
    setOutputs();
}

void LittleSexControl::setOutputs()
{
    //TODO a mettre a jour pour 2012
    //saturation des consignes
    //m_computedVelocityCmd.linear = saturate(m_computedVelocityCmd.linear, -m_vMax, m_vMax);
    //m_computedVelocityCmd.angular = saturate(m_computedVelocityCmd.angular, -m_orderConfig.ANG_VEL_MAX,
     //       m_orderConfig.ANG_VEL_MAX);

    //ROS_WARN("linear=%0.3f angular=%0.3f",m_computedVelocityCmd.linear,m_computedVelocityCmd.angular);

    // On publie la consigne
    //vel_pub_.publish(m_computedVelocityCmd);

    outOrderInError.write(attrOrder->getMode() == MODE_ERROR);
    outOrderFinished.write(isOrderFinished());
    outTwistCmd.write(attrComputedTwistCmd);
}

void LittleSexControl::checkWheelBlocked()
{
    //TODO a mettre a jour pour 2012
//    //si les roues sont bloquées et que le timer est à 0 c'est qu'on vient de commencer à bloquer
//    if( m_wheelBlocked )
//    {
//        if( m_blockTime == 0)
//        {
//            m_blockTime = getTime();
//        }
//        else
//        {
//            double delay = getTime() - m_blockTime;
//            if( delay < 0 ||  delay > WHEEL_BLOCKED_TIMEOUT )
//            {
//                m_wheelBlockedTimeout = true;
//            }
//        }
//    }
//    else
//    {
//        m_blockTime = 0;
//        m_wheelBlockedTimeout = false;
//    }
}

bool LittleSexControl::isOrderFinished()
{
    if (attrOrder->getPass() && attrOrder->getMode() == MODE_PASS)
        return true;

    if (attrOrder->getMode() == MODE_DONE)
        return true;

    return false;
}


bool LittleSexControl::ooSetOrder(shared_ptr<MotionOrder> order)
{
    //TODO tester si y'a pas déjà un order en cours ?

    attrOrder = order;
    return true;
}

void LittleSexControl::createOrocosInterface()
{
    addAttribute("attrComputedTwistCmd",attrComputedTwistCmd);
    addAttribute("attrPosition",attrPosition);

    addPort("inOrder",inOrder)
        .doc("");
    addEventPort("inPosition",inPosition)
        .doc("");

    addPort("outTwistCmd",outTwistCmd)
            .doc("");
    addPort("outOrderFinished",outOrderFinished)
            .doc("");
    addPort("outOrderInError",outOrderInError)
            .doc("");
}
