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
         Twist2D currentTwist,
         MotorState currentMS,
         const UbiquityParams & params,
         const double & dt,
         const double & minStartSpeed,
         Twist2D & acceptableTwist,
         double & quality
    )
{
    //commandes moteurs qui permettent de produire le currentTwist
    MotorState desiredMS;
    //Variable intermédiaires pour les modèle qu'on utilisera pas
    TurretState ioTS;
    SlippageReport oSR;
    quality = -666;

    if( !params.check() )
        return false;



    //cas du départ du robot :  si la mesure est autour de 0 on considère que le robot se déplace quand même un peu vers l'avant pour
    //que le twist définissent bien la configuration de tourelle.
    if( currentTwist.speedNorm() <= minStartSpeed )
    {
        currentMS.driving.left.velocity = minStartSpeed/0.066*2;
        currentMS.driving.right.velocity = minStartSpeed/0.066*2;
        currentMS.driving.rear.velocity = minStartSpeed/0.066*2;
        if( UbiquityKinematics::motors2Twist(currentMS, ioTS, currentTwist, oSR, params) == false )
        {
            Log(ERROR) << "failed to compute forcing speed model";
            quality = -200;
            return false;
        }
        Log(INFO) << "Robot starting, forcing min speed, new Twist=" << currentTwist.toString();
    }


    //cas de l'arrêt du robot : on prend l'état des tourelles actuel, on conserve l'orientation des tourelles et on colle une vitesse nulle
    //ceci revient à ne pas filtrer le twist  les choses se feront toutes seules
    if( desiredTwist.speedNorm() <= minStartSpeed )
    {
        Log(DEBUG) << "Robot is stopping, just let him do his best";
        acceptableTwist = desiredTwist;
        quality = 1;
        return true;
    }


    //before going to dichotomy, check is the desiredTwist is reachable
    if( UbiquityKinematics::twist2Motors(desiredTwist, currentMS, ioTS, desiredMS, params) == false )
    {
        Log(ERROR) << "failed to compute first model";
        quality = -1;
        return false;
    }
    if( isMotorStateReachable(desiredMS, currentMS, params, dt) )
    {
        Log(DEBUG) << "First try is ok";
        acceptableTwist = desiredTwist;
        quality = 1;
        return true;
    }
    //Log(INFO) << "desiredMS : " << desiredMS.toString();

    //the case desiredTwist == currentTwist should be covered before, if we are here it is a bug.
    //we check this twice as it is a singularity of the quality fraction
    if( desiredTwist == currentTwist)
    {
        Log(ERROR) << "assertion desiredTwist != currentTwist failed";
        quality = -10;
        return false;
    }

    //Else we have to use the dichotomy
    Twist2D delta = desiredTwist - currentTwist;
    double k = 0.5;
    Twist2D testedTwist;
    Log(INFO) << "LOOP(init) desiredTwist="<< desiredTwist.toString() << " current=" <<  currentTwist.toString() << " delta=" << delta.toString();
    for(int nbLoops = 2 ; nbLoops < 50 ; nbLoops++  )
    {
        //we take a Twist on the barycentre of desiredTwist to currentTwist weighted by k (the dichotomy is on k)
        testedTwist = delta*k + currentTwist;

        //TODO a virer
        Log(INFO) << "LOOP("<< nbLoops<< ") : testedTwist=" << testedTwist.toString() << " phi=" << testedTwist.speedAngle() << " v=" << testedTwist.speedNorm()<< " k=" << k;

        //compute motor state related to testedTwist
        if( UbiquityKinematics::twist2Motors(testedTwist, currentMS, ioTS, desiredMS, params) == false )
        {
            Log(ERROR) << "fail to compute in loop model loop=" << nbLoops << " k=" << k ;
            quality = -2;
            return false;
        }
        //check if this motor state would be OK. If yes we can try to find a newer Twist closer to desiredTwist,
        //else we should be less aggressive and find a Twist closer to currentTwist
        if( isMotorStateReachable( desiredMS, currentMS, params, dt ) )
        {
            k = k + 1.0/pow(2,nbLoops);
            //we always register the last correct Twist to get it at the end of the loop
            acceptableTwist = testedTwist;
        }
        else
            k = k - 1.0/pow(2,nbLoops);

        //TODO a virer
        //Log(INFO) << "LOOP("<< nbLoops<< ") : desiredMS=" << desiredMS.toString();
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
                << testedTwist.toString() << " k=" << k << " delta=" << delta.toString();
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
