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
        propMaxRho(0.4),
        propMaxRhoAcc(0.7),
        propExpertMaxRho(0.6),
        propExpertMaxRhoAcc(2),
        attrRho(0.0),
        attrPhi(0.0),
        attrDelta(0.0),
        attrVelocityCmdCdg(),
        //TODO periode hardcodee
        m_ovg(0.010),
        m_state()

{
    createOrocosItf();
}

bool IcrSpeedTeleop::configureHook()
{
    bool res = OdsTaskContext::configureHook();

    return res;
}

void IcrSpeedTeleop::updateHook()
{
    OdsTaskContext::updateHook();

    UbiquityParams params;
    double rhoSpeedCmd,phiCmd,deltaCmd;
    double rhoMax = 0.0;
    bool deadMan,power,expert;
    std_msgs::Bool ready;

    //on ne commence pas tant que la strat n'a pas fini son boulot, sinon y'a interférence.
    if( inBootUpDone.readNewest(ready) == NoData || !ready.data )
        return;


    if( inPower.readNewest(power) == NoData )
        power = false;
    if( inDeadMan.readNewest(deadMan) == NoData )
        deadMan = false;

    inRoSpeedCmd.readNewest(rhoSpeedCmd);
    inPhiCmd.readNewest(phiCmd);
    inDeltaCmd.readNewest(deltaCmd);
    inParams.readNewest(params);

    //selection des proprietes en mode normal/expert en fonction de l'etat du bouton "ExpertMode"
    if( inExpertMode.readNewest(expert) != NoData && expert )
    {
        m_ovg.setDynamicLimitations(propExpertMaxRho,propExpertMaxRhoAcc,20);
        rhoMax = propExpertMaxRho;
    }
    else
    {
        m_ovg.setDynamicLimitations(propMaxRho,propMaxRhoAcc,10);
        rhoMax = propMaxRho;

        //on garde les acc expert si on a une vitesse trop grande
        if( fabs(attrRho) > fabs(propMaxRho) )
        {
            m_ovg.setDynamicLimitations(propMaxRho,propExpertMaxRhoAcc,20);
        }
    }

    //filtrage puissance sur les entrees pour adoucir le début de la course joystick
    rhoSpeedCmd = rhoMax*pow(rhoSpeedCmd,5);

    //on applique une commande d'angle aux tourelles que si le robot a suffisamment de vitesse
    if( fabs(rhoSpeedCmd) >= 0.001 )
    {
        attrRho = saturate(rhoSpeedCmd,-rhoMax,rhoMax);
        attrPhi = betweenMinusPiAndPlusPi(-phiCmd-M_PI_2);
        attrDelta = -deadZone(deltaCmd, 0.1, M_PI_2*0.6, 0.1, 0.9);

    }
    //si on ne bouge pas alors il y a rotation pure si le joystick de droite est activé
    else if( fabs(deltaCmd) >= 0.001 )
    {
        attrRho = fabs(saturate(rhoSpeedCmd,-rhoMax,rhoMax));
        attrPhi = -M_PI_2;
        attrDelta = -sign(deltaCmd)*M_PI_2;
    }
    //si tout est a 0 on garde ce qu'on avait
    else
    {
        attrRho = 0;
        attrPhi = attrVelocityCmdCdg.phi();
        attrDelta = attrVelocityCmdCdg.delta();
    }

    if( deadMan == false )
    {
        attrRho = 0;
        m_ovg.setDynamicLimitations(0.0,3,100);
    }

    PosVelAcc reachableRho;
    m_ovg.computeNextStep(attrRho, m_state, reachableRho);
    //attrRho = reachableRho.velocity;
    m_state = reachableRho;

    attrVelocityCmdCdg = ICRSpeed(attrRho,attrPhi,attrDelta);
    //mais le robot se pilote au centre des tourelles :(
    ICRSpeed velocityRef = attrVelocityCmdCdg.transport(params.getChassisCenter().inverse());

    outICRSpeedCmd.write(velocityRef);
}


void IcrSpeedTeleop::createOrocosItf()
{
    addAttribute("attrVelocityCmdCdg",attrVelocityCmdCdg);
    addAttribute("attrRho",attrRho);
    addAttribute("attrPhi",attrPhi);
    addAttribute("attrDelta",attrDelta);
    addProperty("propMaxRho",propMaxRho);
    addProperty("propMaxRhoAcc",propMaxRhoAcc);
    addProperty("propExpertMaxRho",propExpertMaxRho);
    addProperty("propExpertMaxRhoAcc",propExpertMaxRhoAcc);

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

