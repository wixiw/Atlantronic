/*
 * FreeWheel.cpp
 *
 *  Created on: 29 Sept, 2012
 *      Author: ard
 */

#include "FreeWheel.hpp"
#include <rtt/Component.hpp>
#include <iostream>

using namespace RTT;
using namespace arp_math;
using namespace arp_model;
using namespace arp_ods;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_ods::FreeWheel )

FreeWheel::FreeWheel(const std::string& name):
        OdsTaskContext(name),
        attrTwistCmd()
{
    createOrocosItf();
}

bool FreeWheel::configureHook()
{
    bool res = OdsTaskContext::configureHook();

    //res &= getOperation("HmlMonitor", "ooSetMotorPower", m_coSetMotorPower);

    return res;
}

void FreeWheel::updateHook()
{
    OdsTaskContext::updateHook();

    UbiquityParams params;
    bool power;
    std_msgs::Bool ready;

    //on ne commence pas tant que la strat n'a pas fini son boulot, sinon y'a interférence.
    if( inBootUpDone.readNewest(ready) == NoData || !ready.data )
        return;

    if( inPower.readNewest(power) != NoData && power)
        power = true;

    inParams.readNewest(params);

    attrTwistCmd = MathFactory::createTwist2DFromPolarRepr(0,0,0);
    //mais le robot se pilote au centre des tourelles :(
    Twist2D twistRef = attrTwistCmd.transport(params.getChassisCenter().inverse());

    outTwistCmd.write(twistRef);
}


void FreeWheel::createOrocosItf()
{
    addAttribute("attrTwistCmd",attrTwistCmd);

    addPort("inLeftWheelSpeed",inLeftWheelSpeed)
            .doc("");
    addPort("inRightWheelSpeed",inRightWheelSpeed)
            .doc("");

    addPort("inParams",inParams)
            .doc("");
    addPort("inPower",inPower)
            .doc("");
    addPort("inBootUpDone",inBootUpDone)
            .doc("");
    addPort("outTwistCmd",outTwistCmd)
            .doc("");
}

