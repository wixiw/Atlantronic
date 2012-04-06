/*
 * uTest_UK_MotorTurret.hpp
 *
 *  Created on: 04 April 2012
 *      Author: willy
 *
 *      Teste les modèles cinématiques du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityKinematics.hpp"
#include "models/UbiquityParams.hpp"


BOOST_AUTO_TEST_CASE( UK_MotorTurret_ZeroToZeroTest )
{
    arp_model::UbiquityParams params;
    arp_model::MotorState motorsCmd;
    arp_model::TurretState turretCmd;
    arp_model::SteeringMotorVelocities steeringMotorVelocities;
    bool res;

    res = arp_model::UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingVelocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingVelocity,   0 );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingVelocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringPosition,   0 );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringPosition,  0 );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringPosition,   0 );

    res = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, steeringMotorVelocities, motorsCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd.leftDrivingVelocity,     0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightDrivingVelocity,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearDrivingVelocity,     0 );
    BOOST_CHECK_EQUAL( motorsCmd.leftSteeringPosition,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightSteeringPosition,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearSteeringPosition,    0 );
}
