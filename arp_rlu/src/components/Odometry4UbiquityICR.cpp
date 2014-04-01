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
using namespace arp_time;
using namespace arp_model;
using namespace arp_rlu;
using namespace RTT;
using namespace std;

ORO_LIST_COMPONENT_TYPE( arp_rlu::Odometry4UbiquityICR )

Odometry4UbiquityICR::Odometry4UbiquityICR(const std::string& name):
    RluTaskContext(name),
    attrTotalDistanceRunLeft(0.0),
    attrTotalDistanceRunRight(0.0),
    attrTotalDistanceRunRear(0.0)
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

    ArdAbsoluteTime newTime;
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
    measuredICRSpeed.date( newTime );

    outICRSpeed.write(measuredICRSpeed);
    outTwist.write(measuredICRSpeed.twist());
    
        // On estime la position par intégration de l'ICRSpeed courrant;
    Twist2D twist = measuredICRSpeed.twist();
    ArdTimeDelta dt = getTimeDelta(attrLastTime, newTime);
    attrLastTime = newTime;
    vxIntegrator.set(dt, twist.vx());
    vyIntegrator.set(dt, twist.vy());
    vhIntegrator.set(dt, twist.vh());

    // Si on a reçu un cap provenant de l'extérieur, alors on l'applique
    double trueHeading;
    if( RTT::NewData == inTrueHeading.readNewest(trueHeading))
    {
        vhIntegrator.reset(trueHeading);
    }

    // Si on a reçu un cap provenant de l'extérieur, alors on l'applique
    Pose2D externalPose;
    if( RTT::NewData == inTruePose.readNewest(externalPose))
    {
        vxIntegrator.reset(externalPose.x());
        vyIntegrator.reset(externalPose.y());
        vhIntegrator.reset(externalPose.h());
    }

    // On construit la Pose finale
    EstimatedPose2D pose(Pose2D(vxIntegrator.get(), vyIntegrator.get(), vhIntegrator.get()));
    //pose.cov( covariance ); //TODO calculer la covariance
    pose.date( newTime );

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

    double perimeter;
    vector<ICR> icrs(3);
    UbiquityKinematics::findICRFromTurretAngles(attrTurretState, icrs, perimeter,attrParams);
    outICRSphericalPerimeter.write(perimeter);

    //compute total run distance for calibration
    attrTotalDistanceRunLeft += attrTurretState.driving.left.velocity*dt;
    attrTotalDistanceRunRight += attrTurretState.driving.right.velocity*dt;
    attrTotalDistanceRunRear += attrTurretState.driving.rear.velocity*dt;
}

void Odometry4UbiquityICR::ooResetDistanceRun()
{
    attrTotalDistanceRunLeft = 0;
    attrTotalDistanceRunRight = 0;
    attrTotalDistanceRunRear = 0;
}

void Odometry4UbiquityICR::createOrocosInterface()
{
    addAttribute("attrTurretState", attrTurretState);
    addAttribute("attrMotorState", attrMotorState);
    addAttribute("attrParams", attrParams);
    addAttribute("attrLastTime", attrLastTime);
    addAttribute("attrTotalDistanceRunLeft", attrTotalDistanceRunLeft);
    addAttribute("attrTotalDistanceRunRight", attrTotalDistanceRunRight);
    addAttribute("attrTotalDistanceRunRear", attrTotalDistanceRunRear);

    addPort("inTime",inTime)
            .doc("time in second.\n This port is used as trigger.\n This time is used as date of sensors data.");
    addPort("inParams",inParams)
            .doc("UbiquityParams : model parameters");

    addPort("inMotorState",inMotorState)
            .doc("Measures from HML");

    addPort("inTrueHeading",inTrueHeading)
            .doc("Perfect external heading");

    addPort("inTruePose",inTruePose)
            .doc("Perfect external Pose");

    addPort("outICRSpeed",outICRSpeed)
            .doc("T_robot_table_p_robot_r_robot : Twist of robot reference frame relative to table frame, reduced and expressed in robot reference frame.\n It is an EstimatedTwist, so it contains Twist, estimation date (in sec) and covariance matrix.");
    addPort("outPose",outPose)
                .doc("Pose estimated via integration of ICRSpeed and via external heading if available");
    addPort("outTwist",outTwist)
        .doc("Conversion of the outICRSpeed value into a Twist for debug info.");
    addPort("outSlippageDetected",outSlippageDetected)
        .doc("The computation has detection a slippage");

    addPort("outLRiOmega",outLRiOmega).doc("Angular velocity computed from Left and Right turrets velocity measures");
    addPort("outRiReOmega",outRiReOmega).doc("Angular velocity computed from Right and Rear turrets velocity measures");
    addPort("outReLOmega",outReLOmega).doc("Angular velocity computed from Rear and Left turrets velocity measures");
    addPort("outICRSphericalPerimeter",outICRSphericalPerimeter).doc("This value represents the spherical perimeter of the ICR computed from 3 turrets pairs.");

    addOperation("ooResetDistanceRun", &Odometry4UbiquityICR::ooResetDistanceRun, this, OwnThread);
}
