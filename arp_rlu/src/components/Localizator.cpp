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
}


void Localizator::setParams(LocalizatorParams params)
{
    propParams = params;
}

void Localizator::createOrocosInterface()
{
    addProperty("propParams",propParams);

    addEventPort("inScan",inScan)
            .doc("");
    addEventPort("inOdo",inOdo)
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
