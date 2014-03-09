/*
 * KinematicFilter.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#include "KinematicFilter.hpp"
#include <iostream>

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace std;

KinematicFilter::KinematicFilter()
{
    // TODO Auto-generated constructor stub

}

KinematicFilter::~KinematicFilter()
{
    // TODO Auto-generated destructor stub
}

bool KinematicFilter::isMotorStateReachable(const arp_model::MotorState & desiredMS,
                                const arp_model::MotorState & measuredMS,
                                const arp_model::UbiquityParams & params,
                                const double & dt)
{


    //for steering, desired speed must be under threshold
    bool isLeftTurretReachable = fabs(betweenMinusPiAndPlusPi(desiredMS.steering.left.position -measuredMS.steering.left.position)) <= params.getMaxSteeringMotorSpeed()*dt;
    bool isRightTurretReachable = fabs(betweenMinusPiAndPlusPi(desiredMS.steering.right.position-measuredMS.steering.right.position)) <= params.getMaxSteeringMotorSpeed()*dt;
    bool isRearTurretReachable = fabs(betweenMinusPiAndPlusPi(desiredMS.steering.rear.position-measuredMS.steering.rear.position)) <= params.getMaxSteeringMotorSpeed()*dt;
/*
    Log(INFO) << "fabs( "<<desiredMS.steering.left.position <<"-"<<measuredMS.steering.left.position<<") ="<< fabs(desiredMS.steering.left.position -measuredMS.steering.left.position)<<"<="<<params.getMaxSteeringMotorSpeed()*dt<<" : "<<isLeftTurretReachable;
    Log(INFO) << "fabs( "<<desiredMS.steering.right.position <<"-"<<measuredMS.steering.right.position<<") ="<< fabs(desiredMS.steering.right.position -measuredMS.steering.right.position)<<"<="<<params.getMaxSteeringMotorSpeed()*dt<<" : "<<isRightTurretReachable;
    Log(INFO) << "fabs( "<<desiredMS.steering.rear.position <<"-"<<measuredMS.steering.rear.position<<") ="<< fabs(desiredMS.steering.rear.position -measuredMS.steering.rear.position)<<"<="<<params.getMaxSteeringMotorSpeed()*dt<<" : "<<isRearTurretReachable;
*/

    //for driving,   desired speed and acceleration must be under threshold
    bool isLeftDrivingSpeedReachable = fabs(desiredMS.driving.left.velocity-measuredMS.driving.left.velocity) <= params.getMaxDrivingMotorAcc()*dt
            && fabs(desiredMS.driving.left.velocity) <= params.getMaxDrivingSpeed();
    bool isRightDrivingSpeedReachable = fabs(desiredMS.driving.right.velocity-measuredMS.driving.right.velocity) <=  params.getMaxDrivingMotorAcc()*dt
            && fabs(desiredMS.driving.right.velocity) <= params.getMaxDrivingSpeed();
    bool isRearDrivingSpeedReachable = fabs(desiredMS.driving.rear.velocity-measuredMS.driving.rear.velocity) <=  params.getMaxDrivingMotorAcc()*dt
            && fabs(desiredMS.driving.rear.velocity) <= params.getMaxDrivingSpeed();

//    Log(INFO) << "fabs(desiredMS.driving.left.velocity-measuredMS.driving.left.velocity)"<<fabs(desiredMS.driving.left.velocity-measuredMS.driving.left.velocity);
//    Log(INFO) << "params.getMaxDrivingAcc()"<<params.getMaxDrivingAcc();
//    Log(INFO) << "params.getMaxDrivingAcc()*dt"<<params.getMaxDrivingAcc()*dt;
//    Log(INFO) << "reachable: "<<isLeftTurretReachable<< ", "<<isRightTurretReachable<< ", "<<isRearTurretReachable<< ", "<<isLeftDrivingSpeedReachable<< ", "<<isRightDrivingSpeedReachable<< ", "<<isRearDrivingSpeedReachable;

    return isLeftTurretReachable && isRightTurretReachable && isRearTurretReachable
            && isLeftDrivingSpeedReachable && isRightDrivingSpeedReachable && isRearDrivingSpeedReachable;
}
