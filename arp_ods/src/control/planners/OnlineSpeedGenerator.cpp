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

bool OnlineSpeedGenerator::computeNextStep(double targetSpeed, const PosVelAcc & iCurrentState, PosVelAcc & oReachableState)
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

    IP.CurrentPositionVector->VecData      [DOF-1] = iCurrentState.position;
    IP.CurrentVelocityVector->VecData      [DOF-1] = iCurrentState.velocity;
    IP.CurrentAccelerationVector->VecData  [DOF-1] = iCurrentState.acceleration;
    IP.MaxAccelerationVector->VecData      [DOF-1] = m_maxAcc;
    IP.MaxJerkVector->VecData              [DOF-1] = m_maxJerk;
    IP.TargetVelocityVector->VecData       [DOF-1] = targetSpeed;
    IP.SelectionVector->VecData            [DOF-1] = true;

    // Calling the Reflexxes OTG algorithm
    res =  0 > m_RML.RMLVelocity(IP, &OP, m_flags);

    oReachableState.position     = OP.NewPositionVector->VecData[DOF];
    oReachableState.velocity     = OP.NewVelocityVector->VecData[DOF];
    oReachableState.acceleration = OP.NewAccelerationVector->VecData[DOF];

    return res;
}

} /* namespace arp_ods */
