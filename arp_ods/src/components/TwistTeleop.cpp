/*
 * TwistTeleop.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "TwistTeleop.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace arp_math;
using namespace arp_model;
using namespace arp_ods;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::TwistTeleop )

TwistTeleop::TwistTeleop(const std::string& name):
        OdsTaskContext(name),
        propLinearGain(0.3),
        propAngularGain(1),
        propLinearAcc(0.5),
        propAngularAcc(5),
        propExpertLinearGain(1),
        propExpertAngularGain(6),
        propExpertLinearAcc(3),
        propExpertAngularAcc(14),
        propFilter(true),
        attrSpeedCmd(0),
        attrOldSpeedCmd(0),
        attrSpeedDirection(0),
        attrAngularCmd(0),
        attrOldAngularCmd(0),
        attrTwistCmdCdg()
{
    createOrocosItf();
}

bool TwistTeleop::configureHook()
{
    bool res = OdsTaskContext::configureHook();

    res &= getOperation("HmlMonitor", "coSetMotorPower", m_coSetMotorPower);

    return res;
}

void TwistTeleop::updateHook()
{
    OdsTaskContext::updateHook();

    UbiquityParams params;
    double vx,vy,vtheta;
    bool deadMan,power,expert,ready;
    double linAcc, rotAcc, linVel, rotVel;

    //on ne commence pas tant que la strat n'a pas fini son boulot, sinon y'a interférence.
    if( inBootUpDone.readNewest(ready) == NoData || !ready )
        return;


    if( inPower.readNewest(power) == NoData )
        power = false;
    if( inDeadMan.readNewest(deadMan) == NoData )
        deadMan = false;

    inXSpeed.readNewest(vx);
    inYSpeed.readNewest(vy);
    inThetaSpeed.readNewest(vtheta);
    inParams.readNewest(params);


    //selection des proprietes en mode normal/expert
    if( inExpertMode.readNewest(expert) != NoData && expert )
    {
        linAcc = propExpertLinearAcc;
        rotAcc = propExpertAngularAcc;
        linVel = propExpertLinearGain;
        rotVel = propExpertAngularGain;
    }
    else
    {
        linAcc = propLinearAcc;
        rotAcc = propAngularAcc;
        linVel = propLinearGain;
        rotVel = propAngularGain;
    }

    //on filtre les petits mouvements
    if( fabs(vx) <= 0.10 )
        vx = 0.0;
    if( fabs(vy) <= 0.10 )
        vy = 0.0;
    if( fabs(vtheta) <= 0.1 )
        vtheta = 0.0;


    //filtrage puissance sur les entrees pour adoucir le début de la course joystick
    vx = pow(vx,3);
    vy = pow(vy,3);
    vtheta = pow(vtheta,3);


    if (deadMan == true)
    {
        attrAngularCmd = firstDerivateLimitation(-vtheta*rotVel, attrOldAngularCmd, getPeriod(), -rotAcc, rotAcc);
        attrOldAngularCmd = attrAngularCmd;

        attrSpeedCmd = firstDerivateLimitation( linVel*max(fabs(vx),fabs(vy)), attrOldSpeedCmd, getPeriod(), -linAcc, linAcc);
        attrOldSpeedCmd = attrSpeedCmd;

        if( fabs(attrSpeedCmd) <= 0.010 )
        {
            attrSpeedDirection = 0;
        }
        else
        {
            //attention c'est bien atan2(y,x) et j'ai bien mis le x dans le y et inversement, c'est lié au repère du joystick
            attrSpeedDirection= atan2(-vx, -vy);
        }

        //LOG(Info) << "Sending : " << attrSpeedDirection << ", "  << attrAngularCmd << ", " << attrSpeedCmd << endlog();

        if( power == false )
        {
            if( !m_coSetMotorPower(true) )
                LOG(Error) << "Failed to set motor power on" << endlog();
        }
    }
    else
    {
        attrSpeedCmd = 0;
        attrOldSpeedCmd = 0;
        attrAngularCmd = 0;
        attrOldAngularCmd = 0;

        //LOG(Info) << "Not Sending." << endlog();

        if( power == true )
        {
            if( !m_coSetMotorPower(false) )
                LOG(Error) << "Failed to set motor power off" << endlog();
        }
    }


    attrTwistCmdCdg = Twist2D::createFromPolar(attrSpeedCmd,attrSpeedDirection, attrAngularCmd);
    //mais le robot se pilote au centre des tourelles :(
    Twist2D twistRef = attrTwistCmdCdg.transport(params.getChassisCenter().inverse());

    outTwistCmd.write(twistRef);
}


void TwistTeleop::createOrocosItf()
{
    addAttribute("attrSpeedCmd",attrSpeedCmd);
    addAttribute("attrOldSpeedCmd",attrOldSpeedCmd);
    addAttribute("attrSpeedDirection",attrSpeedDirection);
    addAttribute("attrAngularCmd",attrAngularCmd);
    addAttribute("attrOldAngularCmd",attrOldAngularCmd);
    addAttribute("attrTwistCmdCdg",attrTwistCmdCdg);
    addProperty("propLinearGain",propLinearGain);
    addProperty("propAngularGain",propAngularGain);
    addProperty("propLinearAcc",propLinearAcc);
    addProperty("propAngularAcc",propAngularAcc);
    addProperty("propExpertLinearGain",propExpertLinearGain);
    addProperty("propExpertAngularGain",propExpertAngularGain);
    addProperty("propExpertLinearAcc",propExpertLinearAcc);
    addProperty("propExpertAngularAcc",propExpertAngularAcc);
    addProperty("propFilter", propFilter)
        .doc("Set this to true if you want acc/dec filters to operates (so it becomes a closed loop). else it only send command in open loop");

    addPort("inXSpeed",inXSpeed)
            .doc("");
    addPort("inYSpeed",inYSpeed)
            .doc("");
    addPort("inThetaSpeed",inThetaSpeed)
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
    addPort("outTwistCmd",outTwistCmd)
            .doc("");
}

