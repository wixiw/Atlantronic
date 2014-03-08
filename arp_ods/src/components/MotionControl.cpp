/*
 * MotionControl.cpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#include "MotionControl.hpp"
#include <rtt/Component.hpp>
#include "control/orders/Logger.hpp"


using namespace arp_ods;
using namespace orders;
using namespace RTT;
using namespace std;
//using namespace arp_core::log;
using namespace arp_math;

ORO_LIST_COMPONENT_TYPE( arp_ods::MotionControl)

MotionControl::MotionControl(const std::string& name) :
        OdsTaskContext(name), attrVmax_asked(1.0), attrCurrentOrder("default"), m_ICRSpeedBuffer(),m_norder(0)

{
    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
     arp_ods::orders::Logger::InitFile("arp_ods", DEBUG);

    attrOrder=OrderFactory::createDefaultOrder();
    createOrocosInterface();


}

void MotionControl::getInputs()
{
    //faut-il tester que c'est bien mis à jour ?
    EstimatedPose2D position;
    inPosition.readNewest(position);
    attrMotionState.setPosition(position);

    EstimatedICRSpeed speed;
    inCurrentICRSpeed.readNewest(speed);
    attrMotionState.setSpeed(speed);

    inCanPeriod.readNewest(attrCanPeriod);
    inParams.readNewest(attrParams);
}

void MotionControl::updateHook()
{
    OdsTaskContext::updateHook();

    //bufferise inputs
    getInputs();

    //note the ICRSpeed reliazed for this turn
    storeICRSpeed();

    attrCurrentOrder = attrOrder->getTypeString();

    //compute current order mode
    attrOrder->switchMode(attrMotionState);

    // calcule les consignes
    attrOrder->setVmax(attrVmax_asked);

    attrComputedICRSpeedCmd = attrOrder->computeSpeed(attrMotionState, attrCanPeriod);

    /*
     * DEBUG: recuperation des données de l'omnidirect
     */
    if (attrOrder->getType() == OMNIDIRECT2)
    {
        outDEBUG1.write(attrOrder->outDEBUG1);
        outDEBUG2.write(attrOrder->outDEBUG2);
        outDEBUG3.write(attrOrder->outDEBUG3);
        outDEBUG4.write(attrOrder->outDEBUG4);
        outDEBUG5.write(attrOrder->outDEBUG5);
        outDEBUG6.write(attrOrder->outDEBUG6);
        outDEBUG7.write(attrOrder->outDEBUG7);
        outDEBUG8.write(attrOrder->outDEBUG8);
        outDEBUG9.write(attrOrder->outDEBUG9);
        //outDEBUG10.write(attrOrder->outDEBUG10);
    }

    if (attrOrder->getType() == OMNIDIRECT2)
        outSmoothLocNeeded.write(attrOrder->m_smoothLocNeeded);
    else
        outSmoothLocNeeded.write(false);

    //publish computed value
    setOutputs();
}

void MotionControl::stopHook()
{
    attrComputedICRSpeedCmd = ICRSpeed(0, 0, 0);
    setOutputs();
}

void MotionControl::setOutputs()
{
    // no filtering of twist command, as it shall be the job of kinematicbase.

    //si on est en erreur dans un ordre pour lequel elle a du sens on informe au dessus
    outOrderInError.write(attrOrder->getMode() == MODE_ERROR && attrOrder->getType() != STAY);

    //la première fois qu'on a fini on publie
    if (isOrderFinished() && outOrderFinished.getLastWrittenValue() == false)
    {
        outOrderFinished.write(true);
    }

    outICRSpeedCmd.write(attrComputedICRSpeedCmd);
}

void MotionControl::storeICRSpeed()
{
    //TODO ici il suffit juste de regarder ro()
    if (attrMotionState.getSpeed().distanceTo(ICRSpeed(0, 0, 0), 1, 0.2) >= MIN_STORED_TWIST_SPEED)
    {
        m_ICRSpeedBuffer.addICRSpeed(attrMotionState.getSpeed(), attrCanPeriod);
    }
}

bool MotionControl::isOrderFinished()
{
    if (attrOrder->getMode() == MODE_DONE)
    {
        return true;
    }

    return false;
}

bool MotionControl::ooSetOrder(shared_ptr<MotionOrder> order)
{
    /*if( attrOrder->getMode() == MODE_DONE || attrOrder->getMode() == MODE_ERROR)
     {*/
    // l'ordre peut etre casse par l'etage du dessus en cas de detection d'erreur, et un nouvel ordre rempile
    outOrderFinished.write(false);
    LOG(Info) << "Received a new order " << order->getTypeString() << " to go to "
            << order->getEndMotionState().getPosition().toString() << endlog();
    attrOrder = order;
    //TODO
    // oui je sais willy c'est dégueulasse mais éh, c'est pas à RosOsdItf de stocker des informations sur le comportement du robot. c'est a LittleSexcontrol.
    // les ordres devraient etre créés ici c'est ca le probleme
    // et pis RosOdsItf devrait etre une simple passerelle
    // j'aurais pas du mettre le controle de blocage robot la bas
    // mais je l'ai mis car le cassage des ordres se fait la bas
    // moralité refaire l'archi.
    attrOrder->setICRSpeedBuffer(m_ICRSpeedBuffer);

    m_norder++;
    outDEBUG8.write(((double)m_norder)/10.0);
    arp_ods::orders::Log(Info) << "-------------------- order number # ---------------------   >> " <<  m_norder  << endlog();

    return true;
    /* }
     else
     {
     LOG(Error) << "Can't do a new order now, I am busy (in mode "<< attrOrder->getMode()<<"), interrupt properly (with a function that doesn't exists yet ^ ^) " << endlog();
     return false;
     }*/
}

bool MotionControl::ooSetVMax(double vmax)
{
    attrVmax_asked = vmax;
    return true;
}

void MotionControl::createOrocosInterface()
{
    addAttribute("attrComputedICRSpeedCmd", attrComputedICRSpeedCmd);
    addAttribute("attrMotionState", attrMotionState);
    addAttribute("attrOrder", attrOrder);
    addAttribute("attrVmax_asked", attrVmax_asked);
    addAttribute("attrCurrentOrder", attrCurrentOrder);
    addAttribute("attrCanPeriod", attrCanPeriod);
    addAttribute("attrParams", attrParams);

    addPort("inPosition", inPosition).doc("");
    addPort("inCurrentICRSpeed", inCurrentICRSpeed).doc("");
    addPort("inParams", inParams).doc("");
    addPort("inCanPeriod", inCanPeriod).doc("Period computed between 2 SYNC messages by the CAN layer");

    //DEBUG
    addPort("outICRSpeedCmd", outICRSpeedCmd).doc("");
    addPort("outOrderFinished", outOrderFinished).doc("");
    addPort("outOrderInError", outOrderInError).doc("");

    addPort("outDEBUG4", outDEBUG4).doc("");
    addPort("outDEBUG1", outDEBUG1).doc("");
    addPort("outDEBUG2", outDEBUG2).doc("");
    addPort("outDEBUG3", outDEBUG3).doc("");
    addPort("outDEBUG6", outDEBUG6).doc("");
    addPort("outDEBUG5", outDEBUG5).doc("");
    addPort("outDEBUG7", outDEBUG7).doc("");
    addPort("outDEBUG8", outDEBUG8).doc("");
    addPort("outDEBUG9", outDEBUG9).doc("");
    addPort("outDEBUG10", outDEBUG10).doc("");

    addPort("outSmoothLocNeeded", outSmoothLocNeeded).doc("MotionControl needs a localization with no steps");

    addOperation("ooSetOrder", &MotionControl::ooSetOrder, this, OwnThread).doc("Define a new order to do").arg(
            "orderRef", "a reference to the new order to do");

    addOperation("ooSetVMax", &MotionControl::ooSetVMax, this, OwnThread).doc("Define a new max velocity to respect").arg(
            "vmax", "max velocity in m/s");
}
