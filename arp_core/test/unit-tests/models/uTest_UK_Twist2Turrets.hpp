/*
 * uTest_UK_Twist2Turrets.hpp
 *
 *  Created on: 04 April 2012
 *      Author: willy
 *
 *      Teste les modèles cinématiques du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"


BOOST_AUTO_TEST_CASE( UK_Twist2Turrets_NormalizeAngle )
{
      double angle = 0;
      double v = 2.9;
      arp_model::UbiquityKinematics::normalizeDirection(angle,v);
      BOOST_CHECK_EQUAL( angle , 0 );
      BOOST_CHECK_EQUAL( v , 2.9 );

      angle = -2;
      v = 2.9;
      arp_model::UbiquityKinematics::normalizeDirection(angle,v);
      BOOST_CHECK_EQUAL( angle , -2+M_PI );
      BOOST_CHECK_EQUAL( v , -2.9 );

      angle = 2;
      v = 2.9;
      arp_model::UbiquityKinematics::normalizeDirection(angle,v);
      BOOST_CHECK_EQUAL( angle , 2-M_PI );
      BOOST_CHECK_EQUAL( v , -2.9 );
}

BOOST_AUTO_TEST_CASE( UK_Twist2Turrets_ZeroToZeroTest )
{
    arp_model::UbiquityParams params;
    arp_math::Twist2D twist;
    arp_math::Twist2D twistZero;
    arp_model::TurretState turretCmd;
    arp_model::SlippageReport splippage;
    bool res;

    res = arp_model::UbiquityKinematics::twist2Turrets(twist, turretCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretVelocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretVelocity,   0 );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretVelocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   0 );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  0 );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   0 );

    res = arp_model::UbiquityKinematics::turrets2Twist(turretCmd, twist , splippage, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK( twist == twistZero );
}

BOOST_AUTO_TEST_CASE( UK_Twist2Turrets_InverseModel )
{
    arp_model::UbiquityParams params;
    arp_math::Twist2D twist(7.3,0,0);
    arp_math::Twist2D twistZero;
    arp_model::TurretState turretCmd;
    arp_model::SlippageReport splippage;
    bool res;

    res = arp_model::UbiquityKinematics::twist2Turrets(twist, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretVelocity,    7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretVelocity,   7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretVelocity,    7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   0 , 1E-6 );

    arp_math::Twist2D twist1(-7.3,0,0);
    res = arp_model::UbiquityKinematics::twist2Turrets(twist1, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretVelocity,    -7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretVelocity,   -7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretVelocity,    -7.3 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  0 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   0 , 1E-6 );

    arp_math::Twist2D twist2(0,8.2,0);
    res = arp_model::UbiquityKinematics::twist2Turrets(twist2, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretVelocity,    8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretVelocity,   8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretVelocity,    8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   M_PI_2 , 1E-6 );

    arp_math::Twist2D twist3(0,-8.2,0);
    res = arp_model::UbiquityKinematics::twist2Turrets(twist3, turretCmd, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_CLOSE( turretCmd.leftDrivingTurretVelocity,    -8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightDrivingTurretVelocity,   -8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearDrivingTurretVelocity,    -8.2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,   M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,  M_PI_2 , 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,   M_PI_2 , 1E-6 );
}
