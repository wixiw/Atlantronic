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
        attrCurrentOrder("default"),
        m_twistBuffer()

{
    createOrocosInterface();

    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    arp_ods::orders::Logger::InitFile("arp_ods", INFO);
}

void LittleSexControl::getInputs()
{
    //faut-il tester que c'est bien mis à jour ?
    inPosition.readNewest(attrPosition);
    inCurrentTwist.readNewest(attrCurrentTwist);
}

void LittleSexControl::updateHook()
{
    OdsTaskContext::updateHook();

    //bufferise inputs
    getInputs();

    //note the twist reliazed for this turn
    storeTwist();

    attrCurrentOrder = attrOrder->getTypeString();

    //compute current order mode
    attrOrder->switchMode(attrPosition);

    // calcule les consignes
    attrComputedTwistCmd = attrOrder->computeSpeed(attrPosition,attrDt);

    /*
     * DEBUG: recuperation des données de l'omnidirect
     */
    if (attrOrder->getType()==OMNIDIRECT)
    {
        outDEBUGPositionError.write(attrOrder->outDEBUGPositionError);
        outDEBUGLinSpeedCorrection.write(attrOrder->outDEBUGLinSpeedCorrection);
        outDEBUGAngSpeedCorrection.write(attrOrder->outDEBUGAngSpeedCorrection);
        outDEBUGSaturation.write(attrOrder->outDEBUGSaturation);
        outDEBUGNormalizedError.write(attrOrder->outDEBUGNormalizedError);
        outDEBUGMode.write(attrOrder->getMode());
        outDEBUGErrorApproachInit.write(attrOrder->outDEBUGErrorApproachInit);
        outDEBUGErrorApproachCur.write(attrOrder->outDEBUGErrorApproachCur);
    }

    if (attrOrder->getType()==OMNIDIRECT)
        outSmoothLocNeeded.write( attrOrder->m_smoothLocNeeded);
    else
        outSmoothLocNeeded.write( false);


    //publish computed value
    setOutputs();
}

void LittleSexControl::stopHook()
{
    attrComputedTwistCmd=Twist2D(0,0,0);
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

    outTwistCmd.write(attrComputedTwistCmd);
}

void LittleSexControl::storeTwist()
{
    if( attrCurrentTwist.distanceTo(Twist2D(0,0,0),1,0.2) >= MIN_STORED_TWIST_SPEED )
    {
        m_twistBuffer.addTwist(attrCurrentTwist,attrDt);
    }
}

bool LittleSexControl::isOrderFinished()
{
    if (attrOrder->getPass() && attrOrder->getMode() == MODE_PASS)
        return true;

    if (attrOrder->getMode() == MODE_DONE)
    {
        outDEBUGMode.write(attrOrder->getMode());
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
        attrOrder->setTwistBuffer(m_twistBuffer);

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
    addAttribute("attrCurrentTwist",attrCurrentTwist);

    addPort("inPosition",inPosition)
        .doc("");
    addPort("inCurrentTwist",inCurrentTwist)
            .doc("");

    //DEBUG
    addPort("outTwistCmd",outTwistCmd)
            .doc("");
    addPort("outOrderFinished",outOrderFinished)
            .doc("");
    addPort("outOrderInError",outOrderInError)
            .doc("");

    addPort("outDEBUGSaturation",outDEBUGSaturation)
                .doc("");
    addPort("outDEBUGPositionError",outDEBUGPositionError)
            .doc("");
    addPort("outDEBUGLinSpeedCorrection",outDEBUGLinSpeedCorrection)
            .doc("");
    addPort("outDEBUGAngSpeedCorrection",outDEBUGAngSpeedCorrection)
             .doc("");
    addPort("outDEBUGMode",outDEBUGMode)
             .doc("");
    addPort("outDEBUGNormalizedError",outDEBUGNormalizedError)
                 .doc("");
    addPort("outDEBUGErrorApproachInit",outDEBUGErrorApproachInit)
                 .doc("");
    addPort("outDEBUGErrorApproachCur",outDEBUGErrorApproachCur)
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
