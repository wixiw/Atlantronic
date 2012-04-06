/*
 * KinematicFilter.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#include "KinematicFilter.hpp"
#include <models/UbiquityKinematics.hpp>
#include <rtt/Component.hpp>

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

bool KinematicFilter::filterTwist(const Twist2D & desTwist, const Twist2D & currentTwist, const TurretState & turretCurrentState, Twist2D& acceptableTwist, const UbiquityParams & params)
{

    if( !params.check() )
    {
        return false;
    }

    Twist2D slowedTwist;

    // call first filtering
    filterForNonholonomy(desTwist,currentTwist,turretCurrentState,slowedTwist,params);
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

void KinematicFilter::filterForNonholonomy(const Twist2D & inputTwist, const Twist2D & currentTwist, const TurretState & turretCurrentState, Twist2D& outputTwist, const UbiquityParams & params)
{
    // GET SATURATION OF TURRETS TO FULLFILL THE ORDER
    TurretState desTurretCmd;
    UbiquityKinematics::twist2Turrets(inputTwist, desTurretCmd, params);

    // Attention Moumou, j'ai l'impression que tu mélanges moteur et tourelle. (BMO)

    /*double speedLeft=desTurretCmd.leftSteeringTurretPosition-motorsCurrentState.leftSteeringMotorPosition;
    double speedRight=desTurretCmd.rightSteeringTurretPosition-motorsCurrentState.rightSteeringMotorPosition;
    double speedRear=desTurretCmd.rearSteeringTurretPosition-motorsCurrentState.rearSteeringMotorPosition;
    double saturationLeft=(speedLeft-params.getMaxTurretSpeed())/params.getMaxTurretSpeed();
    double saturationRight=(speedRight-params.getMaxTurretSpeed())/params.getMaxTurretSpeed();
    double saturationRear=(speedRear-params.getMaxTurretSpeed())/params.getMaxTurretSpeed();

    double saturationMax=std::max(saturationLeft,std::max(saturationRight,saturationRear));*/

    // FROM THIS SATURATION, DECREASE THE DESIRED TWIST
    Twist2D twistInit;
    Twist2D twistReduced;
    transportToCog(inputTwist,twistInit,params);
    ////////////////////////////////////
    twistReduced=twistInit; //utiliser une fonction "scale" a rajouter dans twist2D
    //////////////////////////////////
    transportToRef(twistReduced,outputTwist,params);

}


void KinematicFilter::filterForConstraints(const Twist2D & inputTwist, const Twist2D & currentTwist, Twist2D& outputTwist, const UbiquityParams & params)
{
    outputTwist=inputTwist;
}

void KinematicFilter::transportToCog(const Twist2D & refTwist, Twist2D& cogTwist, const UbiquityParams & params)
{
    cogTwist = refTwist.transport(params.getChassisCenter());
}

void KinematicFilter::transportToRef(const Twist2D & cogTwist, Twist2D& refTwist, const UbiquityParams & params)
{
    refTwist = cogTwist.transport(params.getChassisCenter().inverse());
}
