/*
 * uTest_UbiquityKinematics.hpp
 *
 *  Created on: 20 April 2012
 *      Author: romain
 *
 *      Teste la coherence des modeles
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityParams.hpp"
#include "models/UbiquityKinematics.hpp"

using namespace arp_model;
using namespace arp_math;


BOOST_AUTO_TEST_CASE( UbiquityKinematics_consistency )
{
    UbiquityParams params;
    TurretState turrets;
    MotorState curmotors;
    MotorState motors;
    SlippageReport slippage;
    Twist2D twist;
    bool success;

    // j'initialise les zeros tourelles au hasard
    params.setLeftTurretZero(1.45);
    params.setRightTurretZero(454.54);
    params.setRearTurretZero(-42.1);

    //une petite marche avant de 1 M/s
    curmotors.driving.left.velocity = 1/(params.getTractionRatio()*params.getLeftWheelDiameter()/2);
    curmotors.driving.right.velocity = 1/(params.getTractionRatio()*params.getRightWheelDiameter()/2);
    curmotors.driving.rear.velocity = 1/(params.getTractionRatio()*params.getRearWheelDiameter()/2);
    curmotors.steering.left.position = 0 + params.getLeftTurretZero()*params.getTurretRatio();
    curmotors.steering.right.position = 0 + params.getRightTurretZero()*params.getTurretRatio();
    curmotors.steering.rear.position = 0 + params.getRearTurretZero()*params.getTurretRatio();

    success = UbiquityKinematics::motors2Twist( curmotors,turrets,twist,slippage,params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( twist.vx() , 1.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.left.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.right.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.rear.position , 0.0 , 1E-6);

    success = UbiquityKinematics::twist2Motors( twist,curmotors,turrets,motors,params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( turrets.steering.left.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.right.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.rear.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( motors.driving.left.velocity , 1/(params.getTractionRatio()*params.getLeftWheelDiameter()/2) , 1E-6);
    BOOST_CHECK_CLOSE( motors.driving.right.velocity , 1/(params.getTractionRatio()*params.getRightWheelDiameter()/2) , 1E-6);
    BOOST_CHECK_CLOSE( motors.driving.rear.velocity , 1/(params.getTractionRatio()*params.getRearWheelDiameter()/2) , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.left.position , 0 + params.getLeftTurretZero()*params.getTurretRatio() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.right.position , 0 + params.getRightTurretZero()*params.getTurretRatio() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.rear.position , 0 + params.getRearTurretZero()*params.getTurretRatio() , 1E-6);


}
