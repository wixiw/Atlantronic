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
        //TODO preriode hardcodee
        m_ovg(0.010),
        m_state()

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
    double rhoSpeedCmd,phiCmd,deltaCmd;
    double rhoMax = 0.0;
    bool deadMan,power,expert, rotation;
    std_msgs::Bool ready;

    //on ne commence pas tant que la strat n'a pas fini son boulot, sinon y'a interférence.
    if( inBootUpDone.readNewest(ready) == NoData || !ready.data )
        return;


    if( inPower.readNewest(power) == NoData )
        power = false;
    if( inDeadMan.readNewest(deadMan) == NoData )
        deadMan = false;
    if( inRotationMode.readNewest(rotation) == NoData )
        rotation = false;

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

    //ajustement du range
    attrDelta = saturate(-pow(deltaCmd,5)*M_PI_2, -M_PI_2, M_PI_2);
    attrRho = saturate(rhoSpeedCmd,-rhoMax,rhoMax);

    //on filtre les petits mouvements pour eviter de laisser les tourelles revenir à 0
    if( fabs(rhoSpeedCmd) >= 0.10 )
    {
        attrPhi = betweenMinusPiAndPlusPi(-phiCmd-M_PI_2);
    }

    if( deadMan == false )
    {
        attrRho = 0;
        m_ovg.setDynamicLimitations(0.0,3,100);
    }
    else
    {
        PosVelAcc reachableRho;
        m_ovg.computeNextStep(attrRho, m_state, reachableRho);
        m_state = reachableRho;
    }

    if( rotation ) // on est dans le mode rotation qui permet de piloter le signe de la rotation sans avoir à tourner les tourelles
    {
        attrPhi = 0.; // en rotation on s'en fout de phi
        attrDelta = sign(attrRho) * M_PI_2;
    }

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
    addPort("inRotationMode",inRotationMode)
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

