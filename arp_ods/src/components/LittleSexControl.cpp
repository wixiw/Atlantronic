/*
 * LittleSexControl.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "LittleSexControl.hpp"
#include <rtt/Component.hpp>
#include "control/orders/Logger.hpp"
using namespace arp_ods;
using namespace orders;
using namespace RTT;
using namespace std;
using namespace arp_core::log;

ORO_LIST_COMPONENT_TYPE( arp_ods::LittleSexControl )

LittleSexControl::LittleSexControl(const std::string& name):
        OdsTaskContext(name),
        attrOrder(orders::defaultOrder),
        attrVMax(1.0),
        attrCurrentOrder("default")
{
    createOrocosInterface();
    arp_ods::orders::Logger::InitFile("arp_ods", INFO);
    //arp_ods::orders::Logger::InitConsole("arp_ods", DEBUG);
}

void LittleSexControl::getInputs()
{
    //faut-il tester que c'est bien mis à jour ?
    inPosition.readNewest(attrPosition);
}

void LittleSexControl::updateHook()
{
    OdsTaskContext::updateHook();

    //bufferise inputs
    getInputs();

    attrCurrentOrder = attrOrder->getTypeString();

    //compute current order mode
    attrOrder->switchMode(attrPosition);

    // calcule les consignes
    //TODO recuperer le dt depuis le dernier appel = inClock - lastInClock
    double dt = 0.01;
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

    //si on est en erreur dans un ordre pour lequel elle a du sens on informe au dessus
    outOrderInError.write(attrOrder->getMode() == MODE_ERROR && attrOrder->getType() != NO_ORDER);

    //la première fois qu'on a finit on publie
    if( isOrderFinished() && outOrderFinished.getLastWrittenValue() == false )
    {
        outOrderFinished.write( true );
    }

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
    if( attrOrder->getMode() == MODE_DONE || attrOrder->getMode() == MODE_ERROR)
    {
        outOrderFinished.write(false);
        LOG(Info) << "Received a new order " << order->getTypeString() << " to go to " << order->getEndPose().toString() << endlog();
        attrOrder = order;
        return true;
    }
    else
    {
        LOG(Error) << "Can't do a new order now, I am busy (in mode "<< attrOrder->getMode()<<"), interrupt properly (with a function that doesn't exists yet ^ ^) " << endlog();
        return false;
    }
}

bool LittleSexControl::ooSetVMax(double vmax)
{
    if( vmax >= 0.0 )
    {
        attrVMax = vmax;
        return true;
    }
    else
    {
        return false;
    }
}

void LittleSexControl::createOrocosInterface()
{
    addAttribute("attrComputedTwistCmd",attrComputedTwistCmd);
    addAttribute("attrPosition",attrPosition);
    addAttribute("attrOrder",attrOrder);
    addAttribute("attrVMax",attrVMax);
    addAttribute("attrCurrentOrder",attrCurrentOrder);


    addEventPort("inPosition",inPosition)
        .doc("");

    addPort("outTwistCmd",outTwistCmd)
            .doc("");
    addPort("outOrderFinished",outOrderFinished)
            .doc("");
    addPort("outOrderInError",outOrderInError)
            .doc("");

    addOperation("ooSetOrder",&LittleSexControl::ooSetOrder, this, OwnThread)
                .doc("Define a new order to do")
                .arg("orderRef","a reference to the new order to do");

    addOperation("ooSetVMax",&LittleSexControl::ooSetVMax, this, OwnThread)
                .doc("Define a new max velocity to respect")
                .arg("vmax","max velocity in m/s");
}
