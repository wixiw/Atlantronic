/*
 * Odometry4UbiquityICR.cpp
 *
 *  Created on: Sept, 29, 2013
 *      Author: ard
 */

#include "Odometry4UbiquityICR.hpp"
#include <rtt/Component.hpp>

using namespace arp_core::log;
using namespace arp_math;
using namespace arp_model;
using namespace arp_rlu;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry4UbiquityICR )

Odometry4UbiquityICR::Odometry4UbiquityICR(const std::string& name):
    RluTaskContext(name)
{
    createOrocosInterface();

    //***WARNING*** Ne pas laisser tourner des logs verbeux sur le robot
    //arp_model::Logger::InitFile("arp_model", WARN);
}

void Odometry4UbiquityICR::updateHook()
{
    RluTaskContext::updateHook();
    ICRSpeed computedICRSpeed;
    SlippageReport report;

    if( RTT::NewData != inTime.readNewest(attrTime))
    {
        LOG( Error ) << "No new data in inTime port : updateHook should not be externally trigger => return" << endlog();
        return;
    }

    if( RTT::NoData == inParams.readNewest(attrParams))
    {
        LOG( Error ) << "No data in inParams port => return" << endlog();
        return;
    }

    if( RTT::NewData != inMotorState.readNewest(attrMotorState))
    {
        LOG( Error ) << "No new data in inMotorState port => return" << endlog();
        return;
    }

    //calcul de l'odometrie (oh oui en une ligne c'est beau)
    if( UbiquityKinematics::motors2ICRSpeed(attrMotorState, attrTurretState, computedICRSpeed, report, attrParams) == false )
    {
        LOG(Error) << "Failed to compute Turrets Cmd" << endlog();
        computedICRSpeed = ICRSpeed();
    }

    outSlippageDetected.write(report);

    //TODO calculer la covariance
    EstimatedICRSpeed measuredICRSpeed(computedICRSpeed);
    //measuredICRSpeed.cov( covariance );
    measuredICRSpeed.date( timespec2Double(attrTime) );

    outICRSpeed.write(measuredICRSpeed);


}


void Odometry4UbiquityICR::createOrocosInterface()
{
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrPose", attrPose);
    addAttribute("attrParams", attrParams);
    addAttribute("attrTime", attrTime);

    addPort("inTime",inTime)
            .doc("time in second.\n This port is used as trigger.\n This time is used as date of sensors data.");
    addPort("inParams",inParams)
            .doc("UbiquityParams : model parameters");

    addPort("inMotorState",inMotorState)
            .doc("Measures from HML");

    addPort("outICRSpeed",outICRSpeed)
            .doc("T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
    addPort("outPose",outPose)
                .doc("Pose estimated via integration of ICRSpedd and via external heading if available");
    addPort("outSlippageDetected",outSlippageDetected)
        .doc("The computation has detection a slippage");
}
