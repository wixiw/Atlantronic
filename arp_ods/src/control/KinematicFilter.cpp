/*
 * KinematicFilter.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#include "KinematicFilter.hpp"
#include <models/UbiquityKinematics.hpp>
#include <rtt/Component.hpp>

using namespace arp_core;
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

bool KinematicFilter::filterTwist(Twist2D const  desTwist, Twist2D const  currentTwist, MotorCommands const motorsCurrentState, Twist2D& acceptableTwist, UbiquityParams const params)
{

    if( !params.check() )
    {
        return false;
    }

    Twist2D slowedTwist;

    // call first filtering
    filterForNonholonomy(desTwist,currentTwist,motorsCurrentState,slowedTwist,params);
    //call second filtering
    filterForConstraints(slowedTwist,currentTwist,acceptableTwist,params);

    //////////////////////////////////////// 2 - A TESTER PUIS RETIRER :  ca doit rien faire
    //je transporte et je reviens
    Twist2D bite;
    Twist2D couille;
    transportToCog(desTwist,bite,params);
    transportToRef(bite,couille,params);
    acceptableTwist=couille;

    //////////////////////////////////////// 1  - A TESTER PUIS RETIRER : ca doit rien faire
    //je ne filtre rien pour l'instant !
    acceptableTwist=desTwist;

    return true;
}

void KinematicFilter::filterForNonholonomy(Twist2D const inputTwist, Twist2D const  currentTwist, MotorCommands const motorsCurrentState, Twist2D& outputTwist, UbiquityParams const params)
{
    // GET DIFFERENCE OF TURRET POSITION BETWEEN NOW AND WHAT IS DESIRED
    TurretCommands desTurretCmd;
    UbiquityKinematics::twist2Turrets(inputTwist, desTurretCmd, params);
    double deltaLeft=desTurretCmd.leftSteeringTurretPosition-motorsCurrentState.leftSteeringMotorPosition;
    double deltaRight=desTurretCmd.rightSteeringTurretPosition-motorsCurrentState.rightSteeringMotorPosition;
    double deltaRear=desTurretCmd.rearSteeringTurretPosition-motorsCurrentState.rearSteeringMotorPosition;

    outputTwist=inputTwist;
}


void KinematicFilter::filterForConstraints(Twist2D const  inputTwist, Twist2D const  currentTwist, Twist2D& outputTwist, UbiquityParams const params)
{
    outputTwist=inputTwist;
}

void KinematicFilter::transportToCog(Twist2D const  refTwist, Twist2D& cogTwist, UbiquityParams const params)
{
    cogTwist = refTwist.transport(params.getChassisCenter());
}

void KinematicFilter::transportToRef(Twist2D const  cogTwist, Twist2D& refTwist, UbiquityParams const params)
{
    refTwist = cogTwist.transport(params.getChassisCenter().inverse());
}
