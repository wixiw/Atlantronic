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
        attrVmax_asked(1.0),
        attrCurrentOrder("default"),
        attrGain(0.2),
        m_ICRSpeedBuffer(),
        OTG()

{
    createOrocosInterface();

    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_ods::orders::Logger::InitFile("arp_ods", DEBUG);
}

void LittleSexControl::getInputs()
{
    //faut-il tester que c'est bien mis à jour ?
    inPosition.readNewest(attrPosition);
    inCurrentICRSpeed.readNewest(attrCurrentICRSpeed);
    inParams.readNewest(attrParams);
}

void LittleSexControl::updateHook()
{
    OdsTaskContext::updateHook();

    //bufferise inputs
    getInputs();

    //note the ICRSpeed reliazed for this turn
    storeICRSpeed();

    attrCurrentOrder = attrOrder->getTypeString();

    //compute current order mode
    attrOrder->switchMode(attrPosition);

    // calcule les consignes
    attrOrder->setVmax(attrVmax_asked);
    attrComputedICRSpeedCmd = attrOrder->computeSpeed(attrPosition,attrParams,attrDt);

    /*
     * DEBUG: recuperation des données de l'omnidirect
     */
    if (attrOrder->getType()==OMNIDIRECT or attrOrder->getType()==OMNIDIRECT2)
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
        outDEBUG10.write(attrOrder->outDEBUG10);


        attrOrder->attrGain=attrGain;
    }

    if (attrOrder->getType()==OMNIDIRECT or attrOrder->getType()==OMNIDIRECT2)
        outSmoothLocNeeded.write( attrOrder->m_smoothLocNeeded);
    else
        outSmoothLocNeeded.write( false);


    //publish computed value
    setOutputs();
}

void LittleSexControl::stopHook()
{
    attrComputedICRSpeedCmd=ICRSpeed(0,0,0);
    setOutputs();
}


void LittleSexControl::setOutputs()
{
    // no filtering of twist command, as it shall be the job of kinematicbase.

    //si on est en erreur dans un ordre pour lequel elle a du sens on informe au dessus
    outOrderInError.write(attrOrder->getMode() == MODE_ERROR && attrOrder->getType() != NO_ORDER);

    //la première fois qu'on a fini on publie
    if( isOrderFinished() && outOrderFinished.getLastWrittenValue() == false )
    {
        outOrderFinished.write( true );
    }

    outICRSpeedCmd.write(attrComputedICRSpeedCmd);
}

void LittleSexControl::storeICRSpeed()
{
    if( attrCurrentICRSpeed.distanceTo(ICRSpeed(0,0,0),1,0.2) >= MIN_STORED_TWIST_SPEED )
    {
        m_ICRSpeedBuffer.addICRSpeed(attrCurrentICRSpeed,attrDt);
    }
}

bool LittleSexControl::isOrderFinished()
{
    if (attrOrder->getPass() && attrOrder->getMode() == MODE_PASS)
        return true;

    if (attrOrder->getMode() == MODE_DONE)
    {
        outDEBUG6.write(attrOrder->getMode());
        return true;
    }

    return false;
}


bool LittleSexControl::ooSetOrder(shared_ptr<MotionOrder> order)
{
    /*if( attrOrder->getMode() == MODE_DONE || attrOrder->getMode() == MODE_ERROR)
    {*/
        // l'ordre peut etre casse par l'etage du dessus en cas de detection d'erreur, et un nouvel ordre rempile
        outOrderFinished.write(false);
        LOG(Info) << "Received a new order " << order->getTypeString() << " to go to " << order->getEndPose().toString() << endlog();
        attrOrder = order;
        // oui je sais willy c'est dégueulasse mais éh, c'est pas à RosOsdItf de stocker des informations sur le comportement du robot. c'est a LittleSexcontrol.
        // les ordres devraient etre créés ici c'est ca le probleme
        // et pis RosOdsItf devrait etre une simple passerelle
        // j'aurais pas du mettre le controle de blocage robot la bas
        // mais je l'ai mis car le cassage des ordres se fait la bas
        // moralité refaire l'archi.
        attrOrder->setICRSpeedBuffer(m_ICRSpeedBuffer);
        attrOrder->setOTG(&OTG);

        return true;
   /* }
    else
    {
        LOG(Error) << "Can't do a new order now, I am busy (in mode "<< attrOrder->getMode()<<"), interrupt properly (with a function that doesn't exists yet ^ ^) " << endlog();
        return false;
    }*/
}

bool LittleSexControl::ooSetVMax(double vmax)
{
    attrVmax_asked = vmax;
    return true;
}

void LittleSexControl::createOrocosInterface()
{
    addAttribute("attrComputedICRSpeedCmd",attrComputedICRSpeedCmd);
    addAttribute("attrPosition",attrPosition);
    addAttribute("attrOrder",attrOrder);
    addAttribute("attrVmax_asked",attrVmax_asked);
    addAttribute("attrCurrentOrder",attrCurrentOrder);
    addAttribute("attrCurrentICRSpeed",attrCurrentICRSpeed);
    addAttribute("attrParams", attrParams);
    addAttribute("attrGain",attrGain);

    addPort("inPosition",inPosition)
        .doc("");
    addPort("inCurrentICRSpeed",inCurrentICRSpeed)
            .doc("");
    addPort("inParams", inParams).doc("");

    //DEBUG
    addPort("outICRSpeedCmd",outICRSpeedCmd)
            .doc("");
    addPort("outOrderFinished",outOrderFinished)
            .doc("");
    addPort("outOrderInError",outOrderInError)
            .doc("");

    addPort("outDEBUG4",outDEBUG4)
                .doc("");
    addPort("outDEBUG1",outDEBUG1)
            .doc("");
    addPort("outDEBUG2",outDEBUG2)
            .doc("");
    addPort("outDEBUG3",outDEBUG3)
             .doc("");
    addPort("outDEBUG6",outDEBUG6)
             .doc("");
    addPort("outDEBUG5",outDEBUG5)
                 .doc("");
    addPort("outDEBUG7",outDEBUG7)
                 .doc("");
    addPort("outDEBUG8",outDEBUG8)
                 .doc("");
    addPort("outDEBUG9",outDEBUG9)
                 .doc("");
    addPort("outDEBUG10",outDEBUG10)
                 .doc("");





    addPort("outSmoothLocNeeded",outSmoothLocNeeded)
    .doc("MotionControl needs a localization with no steps");

    addOperation("ooSetOrder",&LittleSexControl::ooSetOrder, this, OwnThread)
                .doc("Define a new order to do")
                .arg("orderRef","a reference to the new order to do");

    addOperation("ooSetVMax",&LittleSexControl::ooSetVMax, this, OwnThread)
                .doc("Define a new max velocity to respect")
                .arg("vmax","max velocity in m/s");
}
