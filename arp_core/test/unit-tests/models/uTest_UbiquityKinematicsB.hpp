/*
 * uTest_UbiquityKinematics.hpp
 *
 *  Created on: 04 April 2012
 *      Author: willy
 *
 *      Teste les modèles cinématiques du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"

using namespace arp_math;
using namespace arp_core;

BOOST_AUTO_TEST_CASE( UbiquityKinematics_NormalizeAngle )
{
      double angle = 0;
      double v = 2.9;
      UbiquityKinematics::normalizeDirection(angle,v);
      BOOST_CHECK_EQUAL( angle , 0 );
      BOOST_CHECK_EQUAL( v , 2.9 );

      angle = -2;
      v = 2.9;
      UbiquityKinematics::normalizeDirection(angle,v);
      BOOST_CHECK_EQUAL( angle , -2+M_PI );
      BOOST_CHECK_EQUAL( v , -2.9 );

      angle = 2;
      v = 2.9;
      UbiquityKinematics::normalizeDirection(angle,v);
      BOOST_CHECK_EQUAL( angle , 2-M_PI );
      BOOST_CHECK_EQUAL( v , -2.9 );
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_ZeroToZeroTest )
{
    UbiquityParams params;
    Twist2D twist;
    Twist2D twistZero;
    TurretCommands turretCmd;
    Slippage splippage;
    bool res;

    res = UbiquityKinematics::twist2Turrets(twist, turretCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      0 );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   0 );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  0 );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   0 );

    res = UbiquityKinematics::turrets2Twist(turretCmd, twist , splippage, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK( twist == twistZero );
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_InverseModel )
{
    UbiquityParams params;
    Twist2D twist(7.3,0,0);
    Twist2D twistZero;
    TurretCommands turretCmd;
    Slippage splippage;
    bool res;

    res = UbiquityKinematics::twist2Turrets(twist, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretSpeed,       7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretSpeed,      7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretSpeed,       7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   0 , 1E-6 );

    Twist2D twist1(-7.3,0,0);
    res = UbiquityKinematics::twist2Turrets(twist1, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretSpeed,       -7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretSpeed,      -7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretSpeed,       -7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   0 , 1E-6 );

    Twist2D twist2(0,8.2,0);
    res = UbiquityKinematics::twist2Turrets(twist2, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretSpeed,       8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretSpeed,      8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretSpeed,       8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   M_PI_2 , 1E-6 );

    Twist2D twist3(0,-8.2,0);
    res = UbiquityKinematics::twist2Turrets(twist3, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretSpeed,       -8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretSpeed,      -8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretSpeed,       -8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   M_PI_2 , 1E-6 );
}
