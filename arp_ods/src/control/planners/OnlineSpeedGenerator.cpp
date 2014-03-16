/*
 * OnlineSpeedGenerator.cpp
 *
 *  Created on: Mar 15, 2014
 *      Author: ard
 */
#include <math/core>
#include "OnlineSpeedGenerator.hpp"

namespace arp_ods
{

OnlineSpeedGenerator::OnlineSpeedGenerator(double theoricalPeriod):
        m_RML(DOF,theoricalPeriod),
        m_flags(),
        m_maxSpeed(0.0),
        m_maxAcc(0.0),
        m_maxJerk(0.0)
{
        m_flags.SynchronizationBehavior=RMLPositionFlags::NO_SYNCHRONIZATION;

}

OnlineSpeedGenerator::~OnlineSpeedGenerator()
{
    // TODO Auto-generated destructor stub
}

/** Define a new maximal speed in m/s */
void OnlineSpeedGenerator::setMaxSpeed(double vMax)
{
    m_maxSpeed = vMax;
}

/** Define a new maximal acc in m/s2 */
void OnlineSpeedGenerator::setMaxAcc(double accMax)
{
    m_maxAcc = accMax;
}

/** Define a new maximal jerk in m/s3 */
void OnlineSpeedGenerator::setMaxJerk(double jerkMax)
{
    m_maxJerk = jerkMax;
}

/** Define new Dynamic limitations */
void OnlineSpeedGenerator::setDynamicLimitations(double vMax, double accMax, double jerkMax)
{
    setMaxSpeed(vMax);
    setMaxAcc(accMax);
    setMaxJerk(jerkMax);
}

bool OnlineSpeedGenerator::computeNextStep(double targetSpeed, PosVelAcc currentState, PosVelAcc& reachableState)
{
    bool res = false;
    RMLVelocityInputParameters  IP(DOF);
    RMLVelocityOutputParameters OP(DOF);

    //max speed is not a parameter of the reflexxes planner as it doesn't make sense to ask for something forbidden
    //the similar case is the table borders when driving in position which are not provided to reflexxes.
    if( fabs(targetSpeed) > m_maxSpeed )
    {
        targetSpeed = arp_math::sign(targetSpeed)*m_maxSpeed;
    }

    IP.CurrentPositionVector->VecData      [DOF] = currentState.position;
    IP.CurrentVelocityVector->VecData      [DOF] = currentState.velocity;
    IP.CurrentAccelerationVector->VecData  [DOF] = currentState.acceleration;
    IP.MaxAccelerationVector->VecData      [DOF] = m_maxAcc;
    IP.MaxJerkVector->VecData              [DOF] = m_maxJerk;
    IP.TargetVelocityVector->VecData       [DOF] = targetSpeed;
    IP.SelectionVector->VecData            [DOF] = true;

    // Calling the Reflexxes OTG algorithm
    res =  0 > m_RML.RMLVelocity(IP, &OP, m_flags);

    reachableState.position     = OP.NewPositionVector->VecData[DOF];
    reachableState.velocity     = OP.NewVelocityVector->VecData[DOF];
    reachableState.acceleration = OP.NewAccelerationVector->VecData[DOF];

    return res;
}

} /* namespace arp_ods */
