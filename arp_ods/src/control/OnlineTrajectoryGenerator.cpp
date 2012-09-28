/*
 * OnlineTrajectoryGenerator.cpp
 *
 *  Created on: september 2012
 *      Author: Romain Moulin
 */

#include "OnlineTrajectoryGenerator.hpp"

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace std;

OnlineTrajectoryGenerator::OnlineTrajectoryGenerator()
{

    //TODO bouhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
    // OH MON DIEU WILLY QUE C'EST PAS BEAU IL A MIS LA VALEUR DU TEMPS DE CYCLE EN DUR
    // HAAAAANNNNNN
    // bon le probleme c'est que onlinetrajectorygenerator fait un boulot de classe statique
    // mais on est quand meme obligé de le construire avec le temps de cycle

    RML = new ReflexxesAPI(1, 0.02);

    // Creating all relevant objects of the Type IV Reflexxes Motion Library

    IP  =   new RMLPositionInputParameters(     1          );//1 = number of DOF

    OP  =   new RMLPositionOutputParameters(    1          );//1 = number of DOF
}

OnlineTrajectoryGenerator::~OnlineTrajectoryGenerator()
{
delete  RML         ;
}

bool OnlineTrajectoryGenerator::computeNextStep(const PosVelAcc & iStart, const PosVelAcc & iEnd, const double & iMaxVelocity,const double & iMaxAcceleration,const double & iMaxJerk,  PosVelAcc & oNext)
{

        // Variable declarations and definitions

        int   ResultValue =   0;

        // Set-up the input parameters

        IP->CurrentPositionVector->VecData      [0] =    iStart.position      ;

        IP->CurrentVelocityVector->VecData      [0] =    iStart.velocity     ;

        IP->CurrentAccelerationVector->VecData  [0] =   iStart.acceleration      ;

        IP->MaxVelocityVector->VecData          [0] =    iMaxVelocity      ;

        IP->MaxAccelerationVector->VecData      [0] =    iMaxAcceleration     ;

        IP->MaxJerkVector->VecData              [0] =    iMaxJerk      ;

        IP->TargetPositionVector->VecData       [0] =   iEnd.position      ;

        IP->TargetVelocityVector->VecData       [0] =    iEnd.velocity       ;

        IP->SelectionVector->VecData            [0] =   true        ;

        // Calling the Reflexxes OTG algorithm
        ResultValue=   RML->RMLPosition(       *IP
                                            ,   OP
                                            ,   Flags       );

        if (ResultValue < 0) return false; //there was an error

        oNext.position=OP->NewPositionVector->VecData[0];
        oNext.velocity=OP->NewVelocityVector->VecData[0];
        oNext.acceleration=OP->NewAccelerationVector->VecData[0];

        return true;
}

