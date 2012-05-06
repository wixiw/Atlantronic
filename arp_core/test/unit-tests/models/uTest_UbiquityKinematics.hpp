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
#include <stdlib.h>
#include <time.h>
using namespace arp_model;
using namespace arp_math;
using namespace std;

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
    BOOST_CHECK_SMALL( turrets.steering.left.position , 1E-6);
    BOOST_CHECK_SMALL( turrets.steering.right.position , 1E-6);
    BOOST_CHECK_SMALL( turrets.steering.rear.position , 1E-6);

    success = UbiquityKinematics::twist2Motors( twist,curmotors,turrets,motors,params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_SMALL( turrets.steering.left.position , 1E-6);
    BOOST_CHECK_SMALL( turrets.steering.right.position , 1E-6);
    BOOST_CHECK_SMALL( turrets.steering.rear.position , 1E-6);
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

    BOOST_CHECK_SMALL( turrets.steering.left.position , 1E-6);
    BOOST_CHECK_SMALL( turrets.steering.right.position , 1E-6);
    BOOST_CHECK_SMALL( turrets.steering.rear.position , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.left.position , params.getLeftTurretZero() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.right.position , params.getRightTurretZero() , 1E-6);
    BOOST_CHECK_CLOSE( motors.steering.rear.position , params.getRearTurretZero() , 1E-6);


}

void checkTwist2Motor2Twist(double vx, double vy, double vh, UbiquityParams params)
{
    cout << "checkTwist2Motor2Twist " << vx << " " << vy << " " << vh
            << " turrets Zeros : " << params.getLeftTurretZero() << " " << params.getRightTurretZero() << " " << params.getRearTurretZero() << endl;

    TurretState turrets_commanded;
    TurretState turrets_back;
    MotorState curmotors;
    MotorState motors;
    SlippageReport slippage;
    Twist2D twist_commanded;
    Twist2D twist_back;
    bool success;

    twist_commanded=Twist2D(vx,vy,vh);

    success = UbiquityKinematics::twist2Motors( twist_commanded,curmotors,turrets_commanded,motors,params);
    BOOST_CHECK_EQUAL( success , true);

    success = UbiquityKinematics::motors2Twist( motors,turrets_back,twist_back,slippage,params);
    BOOST_CHECK_EQUAL( success , true);

    if( fabs(vx) > 1E-6 )
        BOOST_CHECK_CLOSE(twist_back.vx(), twist_commanded.vx(), 1E-6);
    else
        BOOST_CHECK_SMALL(twist_back.vx(), 1E-6);

    if( fabs(vy) > 1E-6 )
        BOOST_CHECK_CLOSE(twist_back.vy(), twist_commanded.vy(), 1E-6);
    else
        BOOST_CHECK_SMALL(twist_back.vy(), 1E-6);

    if( fabs(vh) > 1E-6 )
        BOOST_CHECK_CLOSE(twist_back.vh(), twist_commanded.vh(), 1E-6);
    else
        BOOST_CHECK_SMALL(twist_back.vh(), 1E-6);


//    BOOST_CHECK_CLOSE(turrets_commanded.steering.left.position, turrets_back.steering.left.position, 1E-6);
//    BOOST_CHECK_CLOSE(turrets_commanded.steering.right.position, turrets_back.steering.right.position, 1E-6);
//    BOOST_CHECK_CLOSE(turrets_commanded.steering.rear.position, turrets_back.steering.rear.position, 1E-6);
//    BOOST_CHECK_CLOSE(turrets_commanded.driving.left.velocity, turrets_back.driving.left.velocity, 1E-6);
//    BOOST_CHECK_CLOSE(turrets_commanded.driving.right.velocity, turrets_back.driving.right.velocity, 1E-6);
//    BOOST_CHECK_CLOSE(turrets_commanded.driving.rear.velocity, turrets_back.driving.rear.velocity, 1E-6);

}

/** Retourne un double aléatoire entre -1 et 1 avec une résolution de 1.33E-4*/
double randSpeed()
{
    return (double)(rand()%30000 - 15000)/15000;
}

/** Retourne un double aléatoire entre -10 et 10 avec une résolution de 1.33E-3*/
double randZeroTurretPos()
{
    return (double)(rand()%30000 - 15000)/1500;
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_rotation_twist2motor2twist )
{
    unsigned int seed = time(NULL);
    srand(seed);
    cout << "Seed for rand number is : " << seed << endl;

    UbiquityParams params;
    params.setLeftTurretZero(0.0);
    params.setRightTurretZero(0.0);
    params.setRearTurretZero(0.0);
    checkTwist2Motor2Twist(1.0, 0, 0, params);
    checkTwist2Motor2Twist(0, 1.0, 0, params);
    checkTwist2Motor2Twist(0, 0, 1.0, params);

    for( int i=0 ; i<50; i++)
    {
        params.setLeftTurretZero(randZeroTurretPos());
        params.setRightTurretZero(randZeroTurretPos());
        params.setRearTurretZero(randZeroTurretPos());
        checkTwist2Motor2Twist(randSpeed(), randSpeed(), randSpeed(), params);
    }
}
