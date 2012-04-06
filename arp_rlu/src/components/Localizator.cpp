/*
 * Localizator.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Localizator.hpp"
#include <rtt/Component.hpp>

using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Localizator )

Localizator::Localizator(const std::string& name):
        RluTaskContext(name)
{
    createOrocosInterface();
}


bool Localizator::initialize(EstimatedPose2D pose)
{
    return false;
}


void Localizator::setParams(LocalizatorParams params)
{
    propParams = params;
}

void Localizator::scanCb(RTT::base::PortInterface* portInterface)
{

}

void Localizator::odoCb(RTT::base::PortInterface* portInterface)
{
    EstimatedTwist2D dummy;
    Pose2D p;

    inOdo.readNewest(dummy);
    outTwist.write(dummy);
    outPose.write(p);
}

void Localizator::updadeHook()
{
    // !!!!!!!! BIG FAT WARNING : l'updateHook est appele apres chaque callback automatiquement par orocos !!!
}

void Localizator::createOrocosInterface()
{
    addProperty("propParams",propParams);

    addEventPort("inScan",inScan, boost::bind(&Localizator::scanCb,this,_1))
            .doc("");
    addEventPort("inOdo",inOdo , boost::bind(&Localizator::odoCb,this,_1))
            .doc("");
    addPort("outPose",outPose)
            .doc("");
    addPort("outTwist",outTwist)
            .doc("");

    addOperation("ooInitialize",&Localizator::initialize, this, OwnThread)
                .doc("")
                .arg("pose","");

    addOperation("ooSetParams",&Localizator::setParams, this, OwnThread)
                .doc("")
                .arg("params","");
}
