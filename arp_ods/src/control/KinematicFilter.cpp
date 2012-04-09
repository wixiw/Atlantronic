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

bool KinematicFilter::filterTwist(const Twist2D & desiredTwist,
         const Twist2D & currentTwist,
         const MotorState & currentMS,
         const UbiquityParams & params,
         const double & dt,
         Twist2D & acceptableTwist,
         double & quality)
{
    if( !params.check() )
        return false;

    quality = -666;

    //before going to dichotomy, check is the desiredTwist is reachable
    MotorState desiredMS;
    TurretState ioTS;
    if( UbiquityKinematics::twist2Motors(desiredTwist, currentMS, ioTS, desiredMS, params) == false )
    {
        Log(ERROR) << "failed to compute first model";
        quality = -1;
        return false;
    }
    if( isMotorStateReachable(desiredMS, currentMS, params, dt) )
    {
        Log(INFO) << "First try is ok";
        acceptableTwist = desiredTwist;
        quality = 1;
        return true;
    }
    //Log(INFO) << "desiredMS : " << desiredMS.toString();

    //the case desiredTwist == currentTwist should be covered before, if we are here it is a bug.
    if( desiredTwist == currentTwist)
    {
        Log(ERROR) << "assertion desiredTwist != currentTwist failed";
        quality = -10;
        return false;
    }

    //Else we have to use the dichotomy
    Twist2D lowBound = currentTwist;
    Twist2D highBound = desiredTwist;
    Twist2D testedTwist;
    for(int nbLoops = 0 ; nbLoops < 10 ; nbLoops++  )
    {
        //we take a Twist half way beetween last acceptable Twist and last not acceptable Twist
        testedTwist = (highBound - lowBound) / 2;

        //TODO a virer
        Log(INFO) << "LOOP("<< nbLoops<< ") : testedTwist="
                << testedTwist.toString() << " lowBound=" << lowBound.toString() << " highBound=" << highBound.toString();

        //compute motor state related to testedTwist
        if( UbiquityKinematics::twist2Motors(testedTwist, currentMS, ioTS, desiredMS, params) == false )
        {
            Log(ERROR) << "fail to compute in loop model loop=" << nbLoops << " low=" << lowBound.toString() << " high=" << highBound.toString();
            quality = -2;
            return false;
        }
        //check if this motor state would be OK. If yes we can try to find a newer Twist closer to desiredTwist,
        //else we should be less aggressive and find a Twist closer to currentTwist
        if( isMotorStateReachable( desiredMS, currentMS, params, dt ) )
        {
            lowBound = testedTwist;
            //we always register the last correct Twist to get it at the end of the loop
            acceptableTwist = testedTwist;
        }
        else
            highBound = testedTwist;

        //TODO a virer
        Log(INFO) << "LOOP("<< nbLoops<< ") : desiredMS=" << desiredMS.toString();
    }

    //gains to uniformize units. we take 20cm for the angle gain.
    //That mean a rotation speed as the same weigth as a linear speed on a turret that would be at 20cm from the reference
    Vector3 coef(0.200,1,1);
    //compute quality index (no division by 0 since we ensured desiredTwist != currentTwist)
    quality = (desiredTwist.distanceTo(currentTwist,coef) - desiredTwist.distanceTo(acceptableTwist,coef))/desiredTwist.distanceTo(currentTwist,coef);

    //we don't accept to stay at the same place when having a command
    if( currentTwist == acceptableTwist && currentTwist != desiredTwist)
    {
        Log(ERROR) << "The solution is to take the last cmd twist ... :( : testedTwist="
                << testedTwist.toString() << " lowBound=" << lowBound.toString() << " highBound=" << highBound.toString();
        Log(INFO) << "acceptableTwist=" << acceptableTwist.toString() << " currentTwist=" << currentTwist.toString() << " desiredTwist=" << desiredTwist.toString();
        quality = -3;
        return false;
    }
    else
        return true;
}

bool KinematicFilter::isMotorStateReachable(const arp_model::MotorState & desiredMS,
                                const arp_model::MotorState & measuredMS,
                                const arp_model::UbiquityParams & params,
                                const double & dt)
{
    //for steerings, desired state must be in [mesure - v*dt; mesure + v*dt]
    bool isLeftTurretReachable = fabs(desiredMS.steering.left.position) <= fabs(measuredMS.steering.left.position) + params.getMaxSteeringSpeed()*dt
            && fabs(measuredMS.steering.left.position) - params.getMaxSteeringSpeed()*dt <= fabs(desiredMS.steering.left.position);
    bool isRightTurretReachable = fabs(desiredMS.steering.right.position) <= fabs(measuredMS.steering.right.position) + params.getMaxSteeringSpeed()*dt
            && fabs(measuredMS.steering.right.position) - params.getMaxSteeringSpeed()*dt <= fabs(desiredMS.steering.right.position);
    bool isRearTurretReachable = fabs(desiredMS.steering.rear.position) <= fabs(measuredMS.steering.rear.position) + params.getMaxSteeringSpeed()*dt
            && fabs(measuredMS.steering.rear.position) - params.getMaxSteeringSpeed()*dt <= fabs(desiredMS.steering.rear.position);
    //TODO tenir compte de l'accélération de tourelle ?

    //for driving, disired state must be in [mesure - v*dt; mesure + v*dt] and slower than max speeds.
    bool isLeftDrivingSpeedReachable = fabs(desiredMS.driving.left.velocity) <= fabs(measuredMS.driving.left.velocity) + params.getMaxDrivingAcc()*dt
            && fabs(measuredMS.driving.left.velocity) - params.getMaxDrivingDec()*dt <= fabs(desiredMS.driving.left.velocity)
            && fabs(desiredMS.driving.left.velocity) <= params.getMaxDrivingSpeed();
    bool isRightDrivingSpeedReachable = fabs(desiredMS.driving.right.velocity) <= fabs(measuredMS.driving.right.velocity) + params.getMaxDrivingAcc()*dt
            && fabs(measuredMS.driving.right.velocity) - params.getMaxDrivingDec()*dt <= fabs(desiredMS.driving.right.velocity)
            && fabs(desiredMS.driving.right.velocity) <= params.getMaxDrivingSpeed();
    bool isRearDrivingSpeedReachable = fabs(desiredMS.driving.rear.velocity) <= fabs(measuredMS.driving.rear.velocity) + params.getMaxDrivingAcc()*dt
            && fabs(measuredMS.driving.rear.velocity) - params.getMaxDrivingDec()*dt <= fabs(desiredMS.driving.rear.velocity)
            && fabs(desiredMS.driving.rear.velocity) <= params.getMaxDrivingSpeed();

    return isLeftTurretReachable && isRightTurretReachable && isRearTurretReachable
            && isLeftDrivingSpeedReachable && isRightDrivingSpeedReachable && isRearDrivingSpeedReachable;
}
