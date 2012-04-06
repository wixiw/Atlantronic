/*
 * Odometry4Ubiquity.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#include "Odometry4Ubiquity.hpp"
#include <rtt/Component.hpp>

using namespace arp_math;
using namespace arp_model;
using namespace arp_rlu;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry4Ubiquity )

Odometry4Ubiquity::Odometry4Ubiquity(const std::string& name):
RluTaskContext(name)
{
    createOrocosInterface();
}

void Odometry4Ubiquity::updateHook()
{
    ARDTaskContext::updateHook();

    if( RTT::NewData != inTime.readNewest(attrTime))
    {
        //WLA->BMO : attention, si on nous appelle via une commande c'est possible que ça trigger l'updateHook
        //tu es egalement triggered après le start() il me semble
        LOG( Warning ) << "No new data in inTime port : updateHook should not be externally trigger => return" << endlog();
        return;
    }

    if( RTT::NoData == inParams.readNewest(attrParams))
    {
        LOG( Warning ) << "No data in inParams port => return" << endlog();
        return;
    }

    if( RTT::NewData != inMotorState.readNewest(attrMotorState))
    {
        LOG( Warning ) << "No new data in inMotorState port => return" << endlog();
        return;
    }

    //calcul de l'odometrie (oh oui en une ligne c'est beau)
    Twist2D conputedTwist;
    SlippageReport report;
    if( UbiquityKinematics::motors2Twist(attrMotorState, conputedTwist, report, attrParams) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
    }

    //TODO il va falloir faire quelque chose pour les covariances
    EstimatedTwist2D measuredTwist = conputedTwist;

    outTwist.write(measuredTwist);
}



void Odometry4Ubiquity::createOrocosInterface()
{
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrTime", attrTime);

    addEventPort("inTime",inTime)
            .doc("time in second.\n This port is used as trigger.\n This time is used as date of sensors data.");
    addPort("inParams",inParams)
            .doc("UbiquityParams : model parameters");

    addPort("inMotorState",inMotorState)
            .doc("Measures from HML");

    addPort("outTwist",outTwist)
            .doc("T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
}
