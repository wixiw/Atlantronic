/*
 * IcrSpeedTeleop.cpp
 *
 *  Created on: Jan 12, 2014
 *      Author: willy
 */

#include "IcrSpeedTeleop.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace arp_math;
using namespace arp_model;
using namespace arp_ods;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::IcrSpeedTeleop )

IcrSpeedTeleop::IcrSpeedTeleop(const std::string& name):
        OdsTaskContext(name),
        propLinearGain(0.3),
        propLinearAcc(0.5),
        propExpertLinearGain(0.5),
        propExpertLinearAcc(1.5),
        propFilter(true),
        attrOldRho(0),
        attrRho(0.0),
        attrPhi(0.0),
        attrDelta(0.0),
        attrVelocityCmdCdg()

{
    createOrocosItf();
}

bool IcrSpeedTeleop::configureHook()
{
    bool res = OdsTaskContext::configureHook();

    //res &= getOperation("HmlMonitor", "ooSetMotorPower", m_coSetMotorPower);

    return res;
}

void IcrSpeedTeleop::updateHook()
{
    OdsTaskContext::updateHook();

    UbiquityParams params;
    double linSpeedCmd,phiCmd,deltaCmd;
    bool deadMan,power,expert;
    std_msgs::Bool ready;
    double linAcc, linVel;

    //on ne commence pas tant que la strat n'a pas fini son boulot, sinon y'a interférence.
    if( inBootUpDone.readNewest(ready) == NoData || !ready.data )
        return;


    if( inPower.readNewest(power) == NoData )
        power = false;
    if( inDeadMan.readNewest(deadMan) == NoData )
        deadMan = false;

    inRoSpeedCmd.readNewest(linSpeedCmd);
    inPhiCmd.readNewest(phiCmd);
    inDeltaCmd.readNewest(deltaCmd);
    inParams.readNewest(params);

    //selection des proprietes en mode normal/expert en fonction de l'etat du bouton "ExpertMode"
    if( inExpertMode.readNewest(expert) != NoData && expert )
    {
        linAcc = propExpertLinearAcc;
        linVel = propExpertLinearGain;
    }
    else
    {
        linAcc = propLinearAcc;
        linVel = propLinearGain;

        //on garde les acc expert si on a une vitesse trop grande
        if( fabs(attrRho) > fabs(propLinearGain) )
        {
            linAcc = propExpertLinearAcc;
            cerr << "Linear vcmd = " << attrRho << endl;
        }
    }

    //filtrage puissance sur les entrees pour adoucir le début de la course joystick
    linSpeedCmd = linVel*pow(linSpeedCmd,5);

    //ajustement du range
    attrDelta = saturate(-pow(deltaCmd,5)*M_PI_2, -M_PI_2, M_PI_2);
    attrRho = saturate(linSpeedCmd,-linVel,linVel);

    //on filtre les petits mouvements pour eviter de laisser les tourelles revenir à 0
    if( fabs(linSpeedCmd) >= 0.10 )
    {
        attrPhi = betweenMinusPiAndPlusPi(-phiCmd-M_PI_2);
    }

    attrRho = firstDerivateLimitation( attrRho, attrOldRho, attrDt , -linAcc, linAcc);

    if( deadMan == false )
    {
        attrRho = 0;
    }

    attrVelocityCmdCdg = ICRSpeed(attrRho,attrPhi,attrDelta);
    //mais le robot se pilote au centre des tourelles :(
    ICRSpeed velocityRef = attrVelocityCmdCdg.transport(params.getChassisCenter().inverse());

    outICRSpeedCmd.write(velocityRef);

    attrOldRho = attrRho;
}


void IcrSpeedTeleop::createOrocosItf()
{
    addAttribute("attrOldRho",attrOldRho);
    addAttribute("attrVelocityCmdCdg",attrVelocityCmdCdg);
    addAttribute("attrRho",attrRho);
    addAttribute("attrPhi",attrPhi);
    addAttribute("attrDelta",attrDelta);
    addProperty("propLinearGain",propLinearGain);
    addProperty("propLinearAcc",propLinearAcc);
    addProperty("propExpertLinearGain",propExpertLinearGain);
    addProperty("propExpertLinearAcc",propExpertLinearAcc);
    addProperty("propFilter", propFilter)
        .doc("Set this to true if you want acc/dec filters to operates (so it becomes a closed loop). else it only send command in open loop");

    addPort("inRoSpeedCmd",inRoSpeedCmd)
            .doc("");
    addPort("inPhiCmd",inPhiCmd)
            .doc("");
    addPort("inDeltaCmd",inDeltaCmd)
            .doc("");
    addPort("inParams",inParams)
            .doc("");
    addPort("inDeadMan",inDeadMan)
            .doc("");
    addPort("inPower",inPower)
            .doc("");
    addPort("inExpertMode",inExpertMode)
            .doc("");
    addPort("inBootUpDone",inBootUpDone)
            .doc("");
    addPort("outICRSpeedCmd",outICRSpeedCmd)
            .doc("");
}

