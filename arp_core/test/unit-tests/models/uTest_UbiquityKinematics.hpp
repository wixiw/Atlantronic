/*
 * uTest_UbiquityKinematicsB.hpp
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

BOOST_AUTO_TEST_CASE( UbiquityKinematicsB_ZeroToZeroTest )
{
    UbiquityParams params;
    MotorCommands motorsCmd;
    TurretCommands turretCmd;
    CouplingSpeeds couplingSpeeds;
    bool res;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      0 );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   0 );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  0 );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   0 );

    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd.leftDrivingMotorSpeed,        0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightDrivingMotorSpeed,       0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearDrivingMotorSpeed,        0 );
    BOOST_CHECK_EQUAL( motorsCmd.leftSteeringMotorPosition,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightSteeringMotorPosition,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearSteeringMotorPosition,    0 );
}
