/*
 * KinematicFilter.cpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#include "KinematicFilter.hpp"
#include <models/UbiquityKinematics.hpp>

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

bool KinematicFilter::filterTwist(Twist2D const  desTwist, Twist2D const  currentTwist, Twist2D& acceptableTwist, UbiquityParams const params)
{

    if( !params.check() )
    {
        return false;
    }

    Twist2D slowedTwist;

    // call first filtering
    filterForNonholonomy(desTwist,currentTwist,slowedTwist,params);
    //call second filtering
    filterForConstraints(slowedTwist,currentTwist,acceptableTwist,params)

    //////////////////////////////////////// 2 - A TESTER PUIS RETIRER :  ca doit rien faire
    //je transporte et je reviens
    Twist2D bite;
    Twist2D couille;
    refTwist2cogTwist(desTwist,bite,params);
    cogTwist2refTwist(bite,couille,params);
    acceptableTwist=couille;

    //////////////////////////////////////// 1  - A TESTER PUIS RETIRER : ca doit rien faire
    //je ne filtre rien pour l'instant !
    acceptableTwist=desTwist;

    return true;
}

// Non Holonomy handling: if the robot is asked a twist with a CIR that it will not reached, then it's no use to let him go fast
void KinematicFilter::filterForNonholonomy(Twist2D const  inputTwist, Twist2D const  currentTwist, Twist2D& outputTwist, UbiquityParams const params)
{
    outputTwist=inputTwist;
}

// Handling of the hardware contraints:
// we known that the current twist would be an acceptable solution.
// so we choose a Twist that is something between the desired wtsit and the current twist
void KinematicFilter::filterForConstraints(Twist2D const  inputTwist, Twist2D const  currentTwist, Twist2D& outputTwist, UbiquityParams const params)
{
    outputTwist=inputTwist;
}

//transport of the twist to what is nice for the filter: twist at center of gravity
void KinematicFilter::refTwist2cogTwist(Twist2D const  refTwist, Twist2D& cogTwist, UbiquityParams const params)
{
    cogTwist = refTwist.transport(params.getChassisCenter());
}

void KinematicFilter::cogTwist2refTwist(Twist2D const  cogTwist, Twist2D& refTwist, UbiquityParams const params)
{
    refTwist = cogTwist.transport(params.getChassisCenter().inverse());
}
