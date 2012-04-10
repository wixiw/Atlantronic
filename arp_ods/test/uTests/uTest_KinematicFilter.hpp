/*
 * uTest_KinematicFilter.cpp
 *
 *  Created on: 08 April 2012
 *      Author: willy
 */

#include "control/KinematicFilter.hpp"
#include <iostream>
using namespace std;
using namespace arp_ods;
using namespace arp_math;
using namespace arp_model;
using namespace arp_core::log;

BOOST_AUTO_TEST_CASE( KinematicFilter_isReachable_driving_success )
{
    MotorState desiredMS;
    MotorState currentMS;
    UbiquityParams params;
    double dt = 0.010;
    bool success;

    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );

    //1/2 de vmax
    dt = 100000; //pour ne pas etre gene par les calculs d'acc
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = params.getMaxDrivingSpeed()/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = params.getMaxDrivingSpeed()/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = params.getMaxDrivingSpeed()/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );

    //-1/2 de vmax
    dt = 100000; //pour ne pas etre gene par les calculs d'acc
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = -params.getMaxDrivingSpeed()/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = -params.getMaxDrivingSpeed()/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = -params.getMaxDrivingSpeed()/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );

    //1/2 acc*dt
    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = params.getMaxDrivingAcc()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = params.getMaxDrivingAcc()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = params.getMaxDrivingAcc()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );

    //-1/2 dec dt
    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = -params.getMaxDrivingDec()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = -params.getMaxDrivingDec()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = -params.getMaxDrivingDec()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
}

BOOST_AUTO_TEST_CASE( KinematicFilter_isReachable_driving_fail )
{
    MotorState desiredMS;
    MotorState currentMS;
    UbiquityParams params;
    double dt = 0.010;
    bool success;

    //vitesse trop haute
    dt = 100000; //pour ne pas etre gene par les calculs d'acc
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = params.getMaxDrivingSpeed()*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = params.getMaxDrivingSpeed()*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = params.getMaxDrivingSpeed()*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );

    //vitesse trop basse
    dt = 100000; //pour ne pas etre gene par les calculs d'acc
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = -params.getMaxDrivingSpeed()*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = -params.getMaxDrivingSpeed()*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = -params.getMaxDrivingSpeed()*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );

    //acceleration trop forte
    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = params.getMaxDrivingAcc()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = params.getMaxDrivingAcc()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = params.getMaxDrivingAcc()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );

    //decceleration trop forte
    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.driving.left.velocity = -params.getMaxDrivingDec()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.right.velocity = -params.getMaxDrivingDec()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.driving.rear.velocity = -params.getMaxDrivingDec()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
}

//---------
BOOST_AUTO_TEST_CASE( KinematicFilter_isReachable_steergin_success )
{
    MotorState desiredMS;
    MotorState currentMS;
    UbiquityParams params;
    double dt = 0.010;
    bool success;

    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.steering.left.position = params.getMaxSteeringSpeed()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.steering.right.position = params.getMaxSteeringSpeed()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.steering.rear.position = params.getMaxSteeringSpeed()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );

    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.steering.left.position = -params.getMaxSteeringSpeed()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.steering.right.position = -params.getMaxSteeringSpeed()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
    desiredMS = MotorState();
    desiredMS.steering.rear.position = -params.getMaxSteeringSpeed()*dt/2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( success );
}

BOOST_AUTO_TEST_CASE( KinematicFilter_isReachable_steering_fail )
{
    MotorState desiredMS;
    MotorState currentMS;
    UbiquityParams params;
    double dt = 0.010;
    bool success;

    //vitesse trop forte
    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.steering.left.position = params.getMaxSteeringSpeed()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.steering.right.position = params.getMaxSteeringSpeed()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.steering.rear.position = params.getMaxSteeringSpeed()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );

    //vitesse negative trop forte
    dt = 0.010;
    desiredMS = MotorState();
    desiredMS.steering.left.position = -params.getMaxSteeringSpeed()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.steering.right.position = -params.getMaxSteeringSpeed()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );
    desiredMS = MotorState();
    desiredMS.steering.rear.position = -params.getMaxSteeringSpeed()*dt*2;
    success = KinematicFilter::isMotorStateReachable( desiredMS, currentMS, params, dt );
    BOOST_CHECK( !success );

}

//----


BOOST_AUTO_TEST_CASE( KinematicFilter_Zero )
{
    arp_math::Twist2D attrTwistCmd;
    arp_math::Twist2D attrCurrentTwist;
    arp_math::Twist2D attrAcceptableTwist;
    arp_model::MotorState attrMotorStateCommand;
    arp_model::MotorState attrMotorsCurrentState;
    arp_model::UbiquityParams attrParams;
    double attrQuality;
    bool res;
    double dt = 0.010;
    double minSpeed;

    res = KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
                                    attrMotorsCurrentState, attrParams,
                                    dt, minSpeed, attrAcceptableTwist, attrQuality);
    BOOST_CHECK( res );
	BOOST_CHECK( attrAcceptableTwist == attrTwistCmd );
	BOOST_CHECK( attrAcceptableTwist == attrCurrentTwist );
    BOOST_CHECK_EQUAL( attrQuality , 1.0 );
}

BOOST_AUTO_TEST_CASE( KinematicFilter_InputOk )
{
    Log(INFO) << "Running KinematicFilter_InputOk";
    arp_model::UbiquityParams attrParams;
    double attrQuality;
    bool res;
    double dt = 0.010;
        double minSpeed = 0.001;
    arp_math::Twist2D attrTwistCmd(1,2.3,0.7);
    arp_math::Twist2D attrCurrentTwist = attrTwistCmd;
    arp_math::Twist2D attrAcceptableTwist;
    arp_model::MotorState attrMotorStateCommand;
    arp_model::MotorState attrMotorsCurrentState;
    MotorState desiredMS;
    TurretState ioTS;

    BOOST_CHECK( UbiquityKinematics::twist2Motors(attrTwistCmd, attrMotorsCurrentState, ioTS, attrMotorsCurrentState, attrParams) );
    Log(INFO) << "ioTS=" << ioTS.toString();
    Log(INFO) << "attrMotorsCurrentState=" << attrMotorsCurrentState.toString();

    res = KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
                                    attrMotorsCurrentState, attrParams,
                                    dt, minSpeed, attrAcceptableTwist, attrQuality);
    BOOST_CHECK( res );
    BOOST_CHECK( attrAcceptableTwist == attrTwistCmd );
    BOOST_CHECK( attrAcceptableTwist == attrCurrentTwist );
    BOOST_CHECK_EQUAL( attrQuality , 1.0 );
}

//demarrage a pi/2
BOOST_AUTO_TEST_CASE( KinematicFilter_StartPI2 )
{
    arp_math::Twist2D attrTwistCmd(0,1,0.0);
    arp_math::Twist2D attrCurrentTwist;
    arp_math::Twist2D attrAcceptableTwist;
    arp_model::MotorState attrMotorStateCommand;
    arp_model::MotorState attrMotorsCurrentState;
    arp_model::UbiquityParams attrParams;
    double attrQuality;
    bool res;
    double dt = 0.010;
    double minSpeed = 0.001;

    res = KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
                                    attrMotorsCurrentState, attrParams,
                                    dt, minSpeed, attrAcceptableTwist, attrQuality);
    BOOST_CHECK( res );
    BOOST_CHECK( attrAcceptableTwist != attrTwistCmd );
    BOOST_CHECK( attrAcceptableTwist != attrCurrentTwist );
    Log(INFO) << "quality=" << attrQuality << " twist=" << attrAcceptableTwist.toString();
}

//arret a pi/2
BOOST_AUTO_TEST_CASE( KinematicFilter_StopPI2 )
{
    arp_math::Twist2D attrTwistCmd(0,0,0.0);
    arp_math::Twist2D attrCurrentTwist(0,1,0.0);;
    arp_math::Twist2D attrAcceptableTwist;
    arp_model::MotorState attrMotorStateCommand;
    arp_model::MotorState attrMotorsCurrentState;
    arp_model::UbiquityParams attrParams;
    double attrQuality;
    bool res;
    double dt = 0.010;
    double minSpeed = 0.001;

    res = KinematicFilter::filterTwist(attrTwistCmd, attrCurrentTwist,
                                    attrMotorsCurrentState, attrParams,
                                    dt, minSpeed, attrAcceptableTwist, attrQuality);
    BOOST_CHECK( res );
    BOOST_CHECK( attrAcceptableTwist == attrTwistCmd );
    BOOST_CHECK( attrAcceptableTwist != attrCurrentTwist );
    BOOST_CHECK( attrQuality == 1 );
    Log(INFO) << "quality=" << attrQuality << " twist=" << attrAcceptableTwist.toString();
}
