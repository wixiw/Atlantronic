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
    arp_model::Logger::InitFile("arp_model", WARN);
}

void Odometry4UbiquityICR::updateHook()
{
    RluTaskContext::updateHook();
    ICRSpeed computedICRSpeed;
    SlippageReport report;

    timespec newTime;
    if( RTT::NewData != inTime.readNewest(newTime))
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
    measuredICRSpeed.date( timespec2Double(newTime) );

    outICRSpeed.write(measuredICRSpeed);
    outTwist.write(measuredICRSpeed.twist());
    
        // On estime la position par intégration de l'ICRSpeed courrant;
    Twist2D twist = measuredICRSpeed.twist();
    double dt = (double)(timespec2Double(newTime) - timespec2Double(attrLastTime));
    attrLastTime = newTime;
    vxIntegrator.set(dt, twist.vx());
    vyIntegrator.set(dt, twist.vy());
    vhIntegrator.set(dt, twist.vh());

    // Si on a reçu un cap provenant de l'extérieur, alors on l'applique
    if( RTT::NewData == inHeading.readNewest(attrHeading))
    {
        vhIntegrator.reset(attrHeading);
    }

    // On construit la Pose finale
    EstimatedPose2D pose(Pose2D(vxIntegrator.get(), vyIntegrator.get(), vhIntegrator.get()));
    //pose.cov( covariance ); //TODO calculer la covariance
    pose.date( timespec2Double(newTime) );

    outPose.write( pose );
    

    Vector3 angularSpeeds;
    if( false == UbiquityKinematics::findAngularSpeedFromOdometry(attrTurretState, angularSpeeds, attrParams) )
    {
        LOG(Error) << "Failed to compute AngularCmds" << endlog();
        outLRiOmega.write(0.0);
        outRiReOmega.write(0.0);
        outReLOmega.write(0.0);
    }
    else
    {
        outLRiOmega.write(angularSpeeds[0]);
        outRiReOmega.write(angularSpeeds[1]);
        outReLOmega.write(angularSpeeds[2]);
    }

}

bool Odometry4UbiquityICR::ooInitialize(double x, double y, double theta)
{
    vxIntegrator.reset(x);
    vyIntegrator.reset(y);
    vhIntegrator.reset(theta);
}


void Odometry4UbiquityICR::createOrocosInterface()
{
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrHeading", attrHeading);
    addAttribute("attrPose", attrPose);
    addAttribute("attrParams", attrParams);
    addAttribute("attrLastTime", attrLastTime);

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
    addPort("outTwist",outTwist)
        .doc("Conversion of the outICRSpeed value into a Twist for debug info.");
    addPort("outSlippageDetected",outSlippageDetected)
        .doc("The computation has detection a slippage");
    addOperation("ooInitialize",&Odometry4UbiquityICR::ooInitialize, this, OwnThread)
        .doc("Initialisation de l'Odométrie")
        .arg("x","m")
        .arg("y","m")
        .arg("theta","rad");

    addPort("outLRiOmega",outLRiOmega).doc("Angular velocity computed from Left and Right turrets velocity measures");
    addPort("outRiReOmega",outRiReOmega).doc("Angular velocity computed from Right and Rear turrets velocity measures");
    addPort("outReLOmega",outReLOmega).doc("Angular velocity computed from Rear and Left turrets velocity measures");
}
