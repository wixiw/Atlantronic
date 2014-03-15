/*
 * OnlineTrajectoryGenerator.cpp
 *
 *  Created on: september 2012
 *      Author: Romain Moulin
 */

#include "OnlineTrajectoryGenerator.hpp"

#include "orders/Logger.hpp"

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace std;

OnlineTrajectoryGenerator::OnlineTrajectoryGenerator():Flags()
{

    //TODO bouhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh
    // OH MON DIEU WILLY QUE C'EST PAS BEAU IL A MIS LA VALEUR DU TEMPS DE CYCLE EN DUR
    // HAAAAANNNNNN
    // bon le probleme c'est que onlinetrajectorygenerator fait un boulot de classe statique
    // mais on est quand meme obligé de le construire avec le temps de cycle

    m_dt = 0.01;

    RML = new ReflexxesAPI(1, m_dt);

    // Creating all relevant objects of the Type IV Reflexxes Motion Library

    IP = new RMLPositionInputParameters(1); //1 = number of DOF

    OP = new RMLPositionOutputParameters(1); //1 = number of DOF

    Flags.SynchronizationBehavior=RMLPositionFlags::NO_SYNCHRONIZATION;
}

OnlineTrajectoryGenerator::~OnlineTrajectoryGenerator()
{
    delete RML;
    delete IP;
    delete OP;

}

bool OnlineTrajectoryGenerator::computeNextStep(const PosVelAcc & iStart, const PosVelAcc & iEnd,
        const double & iMaxVelocity, const double & iMaxAcceleration, const double & iMaxJerk, PosVelAcc & oNext)
{

    arp_ods::orders::Log(DEBUG) << "          >>computeNextStep";

    // Variable declarations and definitions

    int ResultValue = 0;

    // Set-up the input parameters

    IP->CurrentPositionVector->VecData[0] = iStart.position;

    IP->CurrentVelocityVector->VecData[0] = iStart.velocity;

    IP->CurrentAccelerationVector->VecData[0] = iStart.acceleration;

    IP->MaxVelocityVector->VecData[0] = iMaxVelocity;

    IP->MaxAccelerationVector->VecData[0] = iMaxAcceleration;

    IP->MaxJerkVector->VecData[0] = iMaxJerk;

    IP->TargetPositionVector->VecData[0] = iEnd.position;

    IP->TargetVelocityVector->VecData[0] = iEnd.velocity;

    IP->SelectionVector->VecData[0] = true;

    // Calling the Reflexxes OTG algorithm
    ResultValue = RML->RMLPosition(*IP, OP, Flags);

    if (ResultValue < 0)
        {
        arp_ods::orders::Log(INFO) << "               ReflexxesAPI ERROR"<<ResultValue;
        if(ResultValue==ReflexxesAPI::RML_FINAL_STATE_REACHED) arp_ods::orders::Log(INFO) << "               RML_FINAL_STATE_REACHED";
        if(ResultValue==ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES) arp_ods::orders::Log(INFO) << "               RML_ERROR_INVALID_INPUT_VALUES";
        if(ResultValue==ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION) arp_ods::orders::Log(INFO) << "               RML_ERROR_EXECUTION_TIME_CALCULATION";
        if(ResultValue==ReflexxesAPI::RML_ERROR_SYNCHRONIZATION) arp_ods::orders::Log(INFO) << "               RML_ERROR_SYNCHRONIZATION";
        if(ResultValue==ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS) arp_ods::orders::Log(INFO) << "               RML_ERROR_NUMBER_OF_DOFS";
        if(ResultValue==ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION) arp_ods::orders::Log(INFO) << "               RML_ERROR_NO_PHASE_SYNCHRONIZATION";
        if(ResultValue==ReflexxesAPI::RML_ERROR_NULL_POINTER) arp_ods::orders::Log(INFO) << "               RML_ERROR_NULL_POINTER";
        if(ResultValue==ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG) arp_ods::orders::Log(INFO) << "               RML_ERROR_EXECUTION_TIME_TOO_BIG";
        //if(ResultValue==ReflexxesAPI::RML_ERROR_OVERRIDE_OUT_OF_RANGE) arp_ods::orders::Log(INFO) << "               RML_ERROR_OVERRIDE_OUT_OF_RANGE";

        arp_ods::orders::Log(DEBUG) << "          <<computeNextStep";
        return false; //there was an error
        }

    oNext.position = OP->NewPositionVector->VecData[0];
    oNext.velocity = OP->NewVelocityVector->VecData[0];
    oNext.acceleration = OP->NewAccelerationVector->VecData[0];

    arp_ods::orders::Log(DEBUG) << "          <<computeNextStep";
    return true;
}

bool OnlineTrajectoryGenerator::computeNextStepCheap(const PosVelAcc & iStart, const PosVelAcc & iEnd,
        const double & iMaxVelocity, const double & iMaxAcceleration, const double & iMaxJerk, PosVelAcc & oNext)
{
    return true;
}
