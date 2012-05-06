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


BOOST_AUTO_TEST_CASE( UbiquityKinematics_forward_motor2twist2motor )
{
    UbiquityParams params;
    TurretState turrets;
    MotorState curmotors;
    MotorState motors;
    SlippageReport slippage;
    Twist2D twist;
    bool success;

    // j'initialise les zeros tourelles au hasard
    params.setLeftTurretZero(5.8471);
    params.setRightTurretZero(7.17852);
    params.setRearTurretZero(-4.43757);

    //une petite marche avant de 1 M/s
    curmotors.driving.left.velocity = 1/(params.getTractionRatio()*params.getLeftWheelDiameter()/2);
    curmotors.driving.right.velocity = 1/(params.getTractionRatio()*params.getRightWheelDiameter()/2);
    curmotors.driving.rear.velocity = 1/(params.getTractionRatio()*params.getRearWheelDiameter()/2);
    curmotors.steering.left.position = params.getLeftTurretZero();
    curmotors.steering.right.position = params.getRightTurretZero();
    curmotors.steering.rear.position = params.getRearTurretZero();

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
    BOOST_CHECK_CLOSE( motors.steering.left.position , params.getLeftTurretZero() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.right.position , params.getRightTurretZero() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.rear.position , params.getRearTurretZero() , 1E-6);


}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_nullspeed )
{
    UbiquityParams params;
    TurretState turrets;
    MotorState curmotors;
    MotorState motors;
    SlippageReport slippage;
    Twist2D twist;
    bool success;

    // j'initialise les zeros tourelles au hasard
    params.setLeftTurretZero(5.8471);
    params.setRightTurretZero(6.17852);
    params.setRearTurretZero(-4.43757);

    twist=Twist2D(0,0,0);

    curmotors.driving.left.velocity = 0;
    curmotors.driving.right.velocity = 0;
    curmotors.driving.rear.velocity = 0;
    curmotors.steering.left.position = 0;
    curmotors.steering.right.position = 0;
    curmotors.steering.rear.position = 0;

    success = UbiquityKinematics::twist2Motors( twist,curmotors,turrets,motors,params);

    BOOST_CHECK_CLOSE( turrets.steering.left.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.right.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( turrets.steering.rear.position , 0.0 , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.left.position , params.getLeftTurretZero() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.right.position , params.getRightTurretZero() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.rear.position , params.getRearTurretZero() , 1E-6);


}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_rotation_twist2motor2twist )
{
    UbiquityParams params;
        TurretState turrets_commanded;
        TurretState turrets_back;
        MotorState curmotors;
        MotorState motors;
        SlippageReport slippage;
        Twist2D twist_commanded;
        Twist2D twist_back;
        bool success;

        // j'initialise les zeros tourelles au hasard
        params.setLeftTurretZero(5.8471);
        params.setRightTurretZero(7.17852);
        params.setRearTurretZero(-4.43757);

        twist_commanded=Twist2D(0,0,1.0);

        success = UbiquityKinematics::twist2Motors( twist_commanded,curmotors,turrets_commanded,motors,params);
        BOOST_CHECK_EQUAL( success , true);

        success = UbiquityKinematics::motors2Twist( curmotors,turrets_back,twist_back,slippage,params);
        BOOST_CHECK_EQUAL( success , true);

        BOOST_CHECK_CLOSE(twist_back.vx(),0,1E-6);
        BOOST_CHECK_CLOSE(twist_back.vy(),0,1E-6);
        BOOST_CHECK_CLOSE(twist_back.vh(),1.0,1E-6);

}
