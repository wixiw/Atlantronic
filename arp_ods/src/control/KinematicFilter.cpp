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

    Log(INFO) << ">> KinematicFilter::filterTwist() ";
    Log(INFO) << "desiredTwist=                                "<< desiredTwist.toString() << " phi=" << desiredTwist.speedAngle() << " v=" << desiredTwist.speedNorm();
    Log(INFO) << "currentTwist=                                "<< currentTwist.toString() << " phi=" << currentTwist.speedAngle() << " v=" << currentTwist.speedNorm();
    Log(INFO) << "current steering motors                 :    " << "("<< currentMS.steering.left.position << " , " << currentMS.steering.right.position << " , " << currentMS.steering.rear.position << ")";
    Log(INFO) << "--";


    //commandes moteurs qui permettent de produire le currentTwist
    MotorState desiredMS;
    //Variable intermédiaires pour les modèle qu'on utilisera pas
    TurretState ioTS;
    //SlippageReport oSR;
    quality = -666;

    if( !params.check() )
        return false;

/*
    // pour debug: je vais calculer le twist moi meme...
    if( UbiquityKinematics::motors2Twist(currentMS, ioTS, currentTwist, oSR, params) == false )
            {
                Log(ERROR) << "failed to compute forcing speed model";
                quality = -200;
                return false;
            }

    Log(INFO) << "currentTwist a partir moteurs="<< currentTwist.toString() << " phi=" << currentTwist.speedAngle() << " v=" << currentTwist.speedNorm();
    Log(INFO) << "turrets resultantes           " << "("<< ioTS.steering.left.position << " , " << ioTS.steering.right.position << " , " << ioTS.steering.rear.position << ")";
    Log(INFO) << "---";
*/

    //cas du départ du robot :  si la mesure est autour de 0 on considère que le robot se déplace quand même pour
    //que le twist définisse bien la configuration de tourelle.
    /*
    if( currentTwist.speedNorm() <= minStartSpeed )
    {
        currentMS.driving.left.velocity = minStartSpeed/0.066*2;
        //currentMS.driving.right.velocity = minStartSpeed/0.066*2;
        //currentMS.driving.rear.velocity = minStartSpeed/0.066*2;
        if( UbiquityKinematics::motors2Twist(currentMS, ioTS, currentTwist, oSR, params) == false )
        {
            Log(ERROR) << "failed to compute forcing speed model";
            quality = -200;
            return false;
        }

        Log(INFO) << "Robot starting, forcing min speed, new Twist=" << currentTwist.toString() << " phi=" << currentTwist.speedAngle() << " v=" << currentTwist.speedNorm();
        Log(INFO) << "turrets after redefinition:                  " << "("<< ioTS.steering.left.position << " , " << ioTS.steering.right.position << " , " << ioTS.steering.rear.position << ")";
        MotorState updatedMS;
        UbiquityKinematics::twist2Motors(currentTwist, currentMS,ioTS,updatedMS,params);
        Log(INFO) << "turrets after recomputation from new twist:  " << "("<< ioTS.steering.left.position << " , " << ioTS.steering.right.position << " , " << ioTS.steering.rear.position << ")";
        Log(INFO) << "motors after recomputation from new twist:   " << "("<< updatedMS.steering.left.position << " , " << updatedMS.steering.right.position << " , " << updatedMS.steering.rear.position << ")";
        Log(INFO) << "---";
    }
    */

    if( currentTwist.speedNorm() <= minStartSpeed )
    {
        Log(DEBUG) << "Robot is starting, for simplification desired twist is accepted";
        acceptableTwist = desiredTwist;
        quality = 1;
        return true;
    }

    //cas de l'arrêt du robot : on prend l'état des tourelles actuel, on conserve l'orientation des tourelles et on colle une vitesse nulle
    //ceci revient à ne pas filtrer le twist  les choses se feront toutes seules
    if( desiredTwist.speedNorm() <= minStartSpeed )
    {
        Log(DEBUG) << "Robot is stopping, just let him do his best";
        acceptableTwist = desiredTwist;
        quality = 1;
        Log(INFO) << "<< KinematicFilter::filterTwist() ";
        return true;
    }


    //before going to dichotomy, check is the desiredTwist is reachable
    if( UbiquityKinematics::twist2Motors(desiredTwist, currentMS, ioTS, desiredMS, params) == false )
    {
        Log(ERROR) << "failed to compute first model";
        quality = -1;
        Log(INFO) << "<< KinematicFilter::filterTwist() ";
        return false;
    }
    if( isMotorStateReachable(desiredMS, currentMS, params, dt) )
    {
        Log(DEBUG) << "First try is ok";
        acceptableTwist = desiredTwist;
        quality = 1;
        Log(INFO) << "<< KinematicFilter::filterTwist() ";
        return true;
    }
    //Log(INFO) << "desiredMS : " << desiredMS.toString();
    Log(INFO) << "desired twist turrets:      " << "("<< ioTS.steering.left.position << " , " << ioTS.steering.right.position << " , " << ioTS.steering.rear.position << ")";

    //the case desiredTwist == currentTwist should be covered before, if we are here it is a bug.
    //we check this twice as it is a singularity of the quality fraction
    if( desiredTwist == currentTwist)
    {
        Log(ERROR) << "assertion desiredTwist != currentTwist failed";
        quality = -10;
        Log(INFO) << "<< KinematicFilter::filterTwist() ";
        return false;
    }

    //Else we have to use the dichotomy
    Twist2D lowLimit;
    Twist2D upLimit;
    Twist2D delta;
    Twist2D testedTwist;
    //double k = 0.5;

    upLimit=desiredTwist;
    lowLimit=currentTwist;


    //Log(INFO) << "LOOP(init) desiredTwist="<< desiredTwist.toString() << " current=" <<  currentTwist.toString() << " delta=" << delta.toString();

    for(int nbLoops = 2 ; nbLoops < 50 ; nbLoops++  )
    {
        //we take a Twist on the barycentre of desiredTwist to currentTwist weighted by k (the dichotomy is on k)
        delta = (upLimit-lowLimit)*0.5;
        testedTwist = lowLimit+delta;

        //TODO a virer
        //Log(INFO) << "LOOP("<< nbLoops<< ") : testedTwist=" << testedTwist.toString() << " phi=" << testedTwist.speedAngle() << " v=" << testedTwist.speedNorm() ;

        //compute motor state related to testedTwist
        if( UbiquityKinematics::twist2Motors(testedTwist, currentMS, ioTS, desiredMS, params) == false )
        {
            Log(ERROR) << "fail to compute in loop model loop=" << nbLoops ;
            quality = -2;
            Log(INFO) << "<< KinematicFilter::filterTwist() ";
            return false;
        }

        //Log(INFO) << "tested twist  turrets:  " << "("<< ioTS.steering.left.position << " , " << ioTS.steering.right.position << " , " << ioTS.steering.rear.position << ")";

        //check if this motor state would be OK. If yes we can try to find a newer Twist closer to desiredTwist,
        //else we should be less aggressive and find a Twist closer to currentTwist
        if( isMotorStateReachable( desiredMS, currentMS, params, dt ) )
        {
            //Log(INFO) << "GOOOOD...going toward desired";
            //we always register the last correct Twist to get it at the end of the loop
            acceptableTwist = testedTwist;

            lowLimit=testedTwist;
        }
        else
        {
            //Log(INFO) << "BAAAAD...going toward current";
            upLimit=testedTwist;
        }

        //TODO a virer
        //Log(INFO) << "LOOP("<< nbLoops<< ") : desiredMS=" << desiredMS.toString();
    }

    // we take 20cm for the angle gain.
    //That mean a rotation speed as the same weigth as a linear speed on a turret that would be at 20cm from the reference
    //compute quality index (no division by 0 since we ensured desiredTwist != currentTwist)
    quality = (desiredTwist.distanceTo(currentTwist,1.0,0.2) - desiredTwist.distanceTo(acceptableTwist,1.0,0.2))/desiredTwist.distanceTo(currentTwist,1.0,0.2);

    //we don't accept to stay at the same place when having a command
    if( currentTwist == acceptableTwist && currentTwist != desiredTwist)
    {
        Log(ERROR) << "The solution is to take the last cmd twist ... :( : testedTwist="
                << testedTwist.toString() << " delta=" << delta.toString();
        Log(INFO) << "acceptableTwist=" << acceptableTwist.toString() << " currentTwist=" << currentTwist.toString() << " desiredTwist=" << desiredTwist.toString();
        Log(INFO) << "<< KinematicFilter::filterTwist() ";
        quality = -3;
        return false;
    }
    else
        Log(INFO) << "SOLUTION : acceptableTwist " << acceptableTwist.toString();
        Log(INFO) << "<< KinematicFilter::filterTwist() ";
        return true;
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
/*
    bool isLeftDrivingSpeedReachable = fabs(desiredMS.driving.left.velocity-measuredMS.driving.left.velocity) <= params.getMaxDrivingMotorAcc()*dt;
            //&& fabs(desiredMS.driving.left.velocity) <= params.getMaxDrivingSpeed();

    Log(INFO) << "fabs(desiredMS.driving.left.velocity-measuredMS.driving.left.velocity)"<<fabs(desiredMS.driving.left.velocity-measuredMS.driving.left.velocity);
    Log(INFO) << "params.getMaxDrivingAcc()"<<params.getMaxDrivingAcc();
    Log(INFO) << "params.getMaxDrivingAcc()*dt"<<params.getMaxDrivingAcc()*dt;

    bool isRightDrivingSpeedReachable = fabs(desiredMS.driving.right.velocity-measuredMS.driving.right.velocity) <=  params.getMaxDrivingMotorAcc()*dt;
            //&& fabs(desiredMS.driving.right.velocity) <= params.getMaxDrivingSpeed();
    bool isRearDrivingSpeedReachable = fabs(desiredMS.driving.rear.velocity-measuredMS.driving.rear.velocity) <=  params.getMaxDrivingMotorAcc()*dt;
            //&& fabs(desiredMS.driving.rear.velocity) <= params.getMaxDrivingSpeed();
*/

    //Log(INFO) << "reachable: "<<isLeftTurretReachable<< ", "<<isRightTurretReachable<< ", "<<isRearTurretReachable<< ", "<<isLeftDrivingSpeedReachable<< ", "<<isRightDrivingSpeedReachable<< ", "<<isRearDrivingSpeedReachable;

    return isLeftTurretReachable && isRightTurretReachable && isRearTurretReachable;
            //&& isLeftDrivingSpeedReachable && isRightDrivingSpeedReachable && isRearDrivingSpeedReachable;
}
