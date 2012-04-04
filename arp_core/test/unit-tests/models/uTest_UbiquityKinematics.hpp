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

BOOST_AUTO_TEST_CASE( UbiquityKinematics_ZeroToZeroTest )
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

BOOST_AUTO_TEST_CASE( UbiquityKinematics_ZeroTurretOffsetTest )
{
    bool res;

    UbiquityParams params;
    params.setLeftTurretZero(0.1);
    params.setRightTurretZero(0.5);
    params.setRearTurretZero(0.3);
    params.setTurretRatio(0.25);

    MotorCommands motorsCmd;
    TurretCommands turretCmd;
    CouplingSpeeds couplingSpeeds;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,        0 );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,        0 );
    BOOST_CHECK_CLOSE( turretCmd.leftSteeringTurretPosition,    0.1, 1E-6);
    BOOST_CHECK_CLOSE( turretCmd.rightSteeringTurretPosition,   0.5, 1E-6 );
    BOOST_CHECK_CLOSE( turretCmd.rearSteeringTurretPosition,    0.3, 1E-6 );

    //attention turretCmd n'est pas nulle
    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd.leftDrivingMotorSpeed,         0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightDrivingMotorSpeed,        0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearDrivingMotorSpeed,         0 );
    BOOST_CHECK_EQUAL( motorsCmd.leftSteeringMotorPosition,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightSteeringMotorPosition,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearSteeringMotorPosition,    0 );


    MotorCommands motorsCmd2;
    TurretCommands turretCmd2;
    //cette fois turretCmd est nul
    res = UbiquityKinematics::turrets2Motors(turretCmd2, motorsCmd2 , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd2.leftDrivingMotorSpeed,         0 );
    BOOST_CHECK_EQUAL( motorsCmd2.rightDrivingMotorSpeed,        0 );
    BOOST_CHECK_EQUAL( motorsCmd2.rearDrivingMotorSpeed,         0 );
    BOOST_CHECK_CLOSE( motorsCmd2.leftSteeringMotorPosition,     -4*0.1, 1E-6 );
    BOOST_CHECK_CLOSE( motorsCmd2.rightSteeringMotorPosition,    -4*0.5, 1E-6 );
    BOOST_CHECK_CLOSE( motorsCmd2.rearSteeringMotorPosition,     -4*0.3, 1E-6 );

    res = UbiquityKinematics::motors2Turrets(motorsCmd2, turretCmd2, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd2.leftDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd2.rightDrivingTurretSpeed,      0 );
    BOOST_CHECK_EQUAL( turretCmd2.rearDrivingTurretSpeed,       0 );
    BOOST_CHECK_EQUAL( turretCmd2.leftSteeringTurretPosition,   0 );
    BOOST_CHECK_EQUAL( turretCmd2.rightSteeringTurretPosition,  0 );
    BOOST_CHECK_EQUAL( turretCmd2.rearSteeringTurretPosition,   0 );

}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_OneToOneTest )
{
    bool res;

    UbiquityParams params;
    params.setTurretRatio(1);
    params.setTractionRatio(1);
    params.setLeftWheelDiameter(2);
    params.setRightWheelDiameter(2);
    params.setRearWheelDiameter(2);

    MotorCommands motorsCmd;
    motorsCmd.leftDrivingMotorSpeed = 13.5;
    motorsCmd.rightDrivingMotorSpeed = 17.3;
    motorsCmd.rearDrivingMotorSpeed = 12.1;
    motorsCmd.leftSteeringMotorPosition = 9.0;
    motorsCmd.rightSteeringMotorPosition = 14.2;
    motorsCmd.rearSteeringMotorPosition = 18.9;

    TurretCommands turretCmd;
    CouplingSpeeds couplingSpeeds;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       motorsCmd.leftDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      motorsCmd.rightDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       motorsCmd.rearDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );

    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       motorsCmd.leftDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      motorsCmd.rightDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       motorsCmd.rearDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );
}



BOOST_AUTO_TEST_CASE( UbiquityKinematics_WheelDiameterTest )
{
    bool res;

    UbiquityParams params;
    params.setTurretRatio(1);
    params.setTractionRatio(1);
    params.setLeftWheelDiameter(1);
    params.setRightWheelDiameter(1);
    params.setRearWheelDiameter(1);

    MotorCommands motorsCmd;
    motorsCmd.leftDrivingMotorSpeed = -13.5;
    motorsCmd.rightDrivingMotorSpeed = -17.3;
    motorsCmd.rearDrivingMotorSpeed = -12.1;
    motorsCmd.leftSteeringMotorPosition = -9.0;
    motorsCmd.rightSteeringMotorPosition = -14.2;
    motorsCmd.rearSteeringMotorPosition = -18.9;

    TurretCommands turretCmd;
    CouplingSpeeds couplingSpeeds;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( 2*turretCmd.leftDrivingTurretSpeed,       motorsCmd.leftDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( 2*turretCmd.rightDrivingTurretSpeed,      motorsCmd.rightDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( 2*turretCmd.rearDrivingTurretSpeed,       motorsCmd.rearDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );

    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( 2*turretCmd.leftDrivingTurretSpeed,       motorsCmd.leftDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( 2*turretCmd.rightDrivingTurretSpeed,      motorsCmd.rightDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( 2*turretCmd.rearDrivingTurretSpeed,       motorsCmd.rearDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_RatiosTests )
{
    bool res;

    UbiquityParams params;
    params.setTurretRatio(2);
    params.setTractionRatio(3);
    params.setLeftWheelDiameter(2);
    params.setRightWheelDiameter(2);
    params.setRearWheelDiameter(2);

    MotorCommands motorsCmd;
    motorsCmd.leftDrivingMotorSpeed = -1643.5;
    motorsCmd.rightDrivingMotorSpeed = 1357.3;
    motorsCmd.rearDrivingMotorSpeed = -6121.1;
    motorsCmd.leftSteeringMotorPosition = -789.0;
    motorsCmd.rightSteeringMotorPosition = -9164.2;
    motorsCmd.rearSteeringMotorPosition = 5198.9;

    TurretCommands turretCmd;
    CouplingSpeeds couplingSpeeds;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       3*motorsCmd.leftDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      3*motorsCmd.rightDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       3*motorsCmd.rearDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   2*motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  2*motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   2*motorsCmd.rearSteeringMotorPosition );

    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       3*motorsCmd.leftDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      3*motorsCmd.rightDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       3*motorsCmd.rearDrivingMotorSpeed );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   2*motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  2*motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   2*motorsCmd.rearSteeringMotorPosition );
}


BOOST_AUTO_TEST_CASE( UbiquityKinematics_CouplingTests )
{
    bool res;

    UbiquityParams params;
    params.setTurretRatio(0.25);
    params.setTractionRatio(1);
    params.setLeftWheelDiameter(2);
    params.setRightWheelDiameter(2);
    params.setRearWheelDiameter(2);

    MotorCommands motorsCmd;
    TurretCommands turretCmd;

    CouplingSpeeds couplingSpeeds;
    couplingSpeeds.leftSteeringMotorSpeed = 1;
    couplingSpeeds.rightSteeringMotorSpeed = 3;
    couplingSpeeds.rearSteeringMotorSpeed = 5;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.leftDrivingTurretSpeed,       0.25 );
    BOOST_CHECK_EQUAL( turretCmd.rightDrivingTurretSpeed,      3*0.25 );
    BOOST_CHECK_EQUAL( turretCmd.rearDrivingTurretSpeed,       5*0.25 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );

    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd.leftDrivingMotorSpeed,       0.0 );
    BOOST_CHECK_EQUAL( motorsCmd.rightDrivingMotorSpeed,      0.0 );
    BOOST_CHECK_EQUAL( motorsCmd.rearDrivingMotorSpeed,       0.0 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_CouplingAndWheelTests )
{
    bool res;

    UbiquityParams params;
    params.setTurretRatio(0.25);
    params.setTractionRatio(1);
    params.setLeftWheelDiameter(1);
    params.setRightWheelDiameter(1);
    params.setRearWheelDiameter(1);

    MotorCommands motorsCmd;
    TurretCommands turretCmd;

    CouplingSpeeds couplingSpeeds;
    couplingSpeeds.leftSteeringMotorSpeed = 1;
    couplingSpeeds.rightSteeringMotorSpeed = 3;
    couplingSpeeds.rearSteeringMotorSpeed = 5;

    res = UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( 2*turretCmd.leftDrivingTurretSpeed,       0.25 );
    BOOST_CHECK_EQUAL( 2*turretCmd.rightDrivingTurretSpeed,      3*0.25 );
    BOOST_CHECK_EQUAL( 2*turretCmd.rearDrivingTurretSpeed,       5*0.25 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );

    res = UbiquityKinematics::turrets2Motors(turretCmd, motorsCmd , couplingSpeeds, params);
    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( 2*motorsCmd.leftDrivingMotorSpeed,       0.0 );
    BOOST_CHECK_EQUAL( 2*motorsCmd.rightDrivingMotorSpeed,      0.0 );
    BOOST_CHECK_EQUAL( 2*motorsCmd.rearDrivingMotorSpeed,       0.0 );
    BOOST_CHECK_EQUAL( turretCmd.leftSteeringTurretPosition,   motorsCmd.leftSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rightSteeringTurretPosition,  motorsCmd.rightSteeringMotorPosition );
    BOOST_CHECK_EQUAL( turretCmd.rearSteeringTurretPosition,   motorsCmd.rearSteeringMotorPosition );
}
