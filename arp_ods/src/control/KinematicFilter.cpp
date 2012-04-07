/*
 * KinematicFilter.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#include "KinematicFilter.hpp"

using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;

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

    quality = 0;

    SteeringMotorVelocities SMV;
    SMV.leftSteeringVelocity = currentMS.leftSteeringVelocity;
    SMV.rightSteeringVelocity = currentMS.rightSteeringVelocity;
    SMV.rearSteeringVelocity = currentMS.rearSteeringVelocity;

    //before going to dichotomy, check is the desiredTwist is reachable
    MotorState desiredMS;
    if( UbiquityKinematics::twist2Motors(desiredTwist, SMV, desiredMS, params) == false )
        return false;
    if( isMotorStateReachable(desiredMS, currentMS, params, dt) )
    {
        acceptableTwist = desiredTwist;
        return true;
    }


    //Else we have to use the dichotomy
    Twist2D lowBound = currentTwist;
    Twist2D highBound = desiredTwist;
    Twist2D testedTwist;
    for(int nbLoops = 0 ; nbLoops < 10 ; nbLoops++  )
    {
        //we take a Twist half way beetween last acceptable Twist and last not acceptable Twist
        testedTwist = (highBound - lowBound) / 2;
        //compute motor state related to testedTwist
        if( UbiquityKinematics::twist2Motors(testedTwist, SMV, desiredMS, params) == false )
            return false;
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
    }

    //gains to uniformize units. we take 20cm for the angle gain.
    //That mean a rotation speed as the same weigth as a linear speed on a turret that would be at 20cm from the reference
    Vector3 coef(0.200,1,1);
    //compute quality index
    quality = (desiredTwist.distanceTo(currentTwist,coef) - desiredTwist.distanceTo(acceptableTwist,coef))/desiredTwist.distanceTo(currentTwist,coef);

    //we don't accept to stay at the same place
    if( currentTwist == acceptableTwist )
        return false;
    else
        return true;
}

bool KinematicFilter::isMotorStateReachable(const arp_model::MotorState & desiredMS,
                                const arp_model::MotorState & measuredMS,
                                const arp_model::UbiquityParams & params,
                                const double & dt)
{
    //for steerings, desired state must be in [mesure - v*dt; mesure + v*dt]
    bool isLeftTurretReachable = fabs(desiredMS.leftSteeringPosition) <= fabs(measuredMS.leftSteeringPosition) + params.getMaxSteeringSpeed()*dt
            && fabs(measuredMS.leftSteeringPosition) - params.getMaxSteeringSpeed()*dt <= fabs(desiredMS.leftSteeringPosition);
    bool isRightTurretReachable = fabs(desiredMS.rightSteeringPosition) <= fabs(measuredMS.rightSteeringPosition) + params.getMaxSteeringSpeed()*dt
            && fabs(measuredMS.rightSteeringPosition) - params.getMaxSteeringSpeed()*dt <= fabs(desiredMS.rightSteeringPosition);
    bool isRearTurretReachable = fabs(desiredMS.rearSteeringPosition) <= fabs(measuredMS.rearSteeringPosition) + params.getMaxSteeringSpeed()*dt
            && fabs(measuredMS.rearSteeringPosition) - params.getMaxSteeringSpeed()*dt <= fabs(desiredMS.rearSteeringPosition);
    //TODO tenir compte de l'accélération de tourelle ?

    //for driving, disired state must be in [mesure - v*dt; mesure + v*dt] and slower than max speeds.
    bool isLeftDrivingSpeedReachable = fabs(desiredMS.leftDrivingVelocity) <= fabs(measuredMS.leftDrivingVelocity) + params.getMaxDrivingAcc()*dt
            && fabs(measuredMS.leftDrivingVelocity) - params.getMaxDrivingDec()*dt <= fabs(desiredMS.leftDrivingVelocity)
            && fabs(desiredMS.leftDrivingVelocity) <= params.getMaxDrivingSpeed();
    bool isRightDrivingSpeedReachable = fabs(desiredMS.rightDrivingVelocity) <= fabs(measuredMS.rightDrivingVelocity) + params.getMaxDrivingAcc()*dt
            && fabs(measuredMS.rightDrivingVelocity) - params.getMaxDrivingDec()*dt <= fabs(desiredMS.rightDrivingVelocity)
            && fabs(desiredMS.rightDrivingVelocity) <= params.getMaxDrivingSpeed();
    bool isRearDrivingSpeedReachable = fabs(desiredMS.rearDrivingVelocity) <= fabs(measuredMS.rearDrivingVelocity) + params.getMaxDrivingAcc()*dt
            && fabs(measuredMS.rearDrivingVelocity) - params.getMaxDrivingDec()*dt <= fabs(desiredMS.rearDrivingVelocity)
            && fabs(desiredMS.rearDrivingVelocity) <= params.getMaxDrivingSpeed();

    return isLeftTurretReachable && isRightTurretReachable && isRearTurretReachable
            && isLeftDrivingSpeedReachable && isRightDrivingSpeedReachable && isRearDrivingSpeedReachable;
}
