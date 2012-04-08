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
    arp_model::MotorState steeringMotorVelocities;
    bool res;

    res = arp_model::UbiquityKinematics::motors2Turrets(motorsCmd, turretCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( turretCmd.driving.left.velocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.driving.right.velocity,   0 );
    BOOST_CHECK_EQUAL( turretCmd.driving.rear.velocity,    0 );
    BOOST_CHECK_EQUAL( turretCmd.steering.left.position,   0 );
    BOOST_CHECK_EQUAL( turretCmd.steering.right.position,  0 );
    BOOST_CHECK_EQUAL( turretCmd.steering.rear.position,   0 );

    res = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, steeringMotorVelocities, motorsCmd, params);

    BOOST_CHECK_EQUAL( res , true);
    BOOST_CHECK_EQUAL( motorsCmd.driving.left.velocity,     0 );
    BOOST_CHECK_EQUAL( motorsCmd.driving.right.velocity,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.driving.rear.velocity,     0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    0 );
}

BOOST_AUTO_TEST_CASE( UK_MotorTurret_turret2Motor_translation )
{
    arp_model::UbiquityParams params;
    arp_model::MotorState motorsCmd;
    arp_model::TurretState turretCmd;
    arp_model::MotorState motorsMeasure;
    bool success;

    //marche avant traction
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.6;
    turretCmd.driving.rear.velocity = 0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    0 );

    //marche AR traction
    turretCmd.driving.left.velocity = -0.5;
    turretCmd.driving.right.velocity = -0.6;
    turretCmd.driving.rear.velocity = -0.7;
    turretCmd.steering.left.position = 0;
    turretCmd.steering.right.position = 0;
    turretCmd.steering.rear.position = 0;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     -0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    -0.6*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     -0.7*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_EQUAL( motorsCmd.steering.left.position,    0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.right.position,   0 );
    BOOST_CHECK_EQUAL( motorsCmd.steering.rear.position,    0 );

    //translation de P1/4 à gauche en AV
    turretCmd.driving.left.velocity = 0.5;
    turretCmd.driving.right.velocity = 0.5;
    turretCmd.driving.rear.velocity = 0.5;
    turretCmd.steering.left.position = M_PI/4;
    turretCmd.steering.right.position = M_PI/4;
    turretCmd.steering.rear.position = M_PI/4;
    success = arp_model::UbiquityKinematics::turrets2Motors(turretCmd, motorsMeasure, motorsCmd, params);
    BOOST_CHECK_EQUAL( success , true);
    BOOST_CHECK_CLOSE( motorsCmd.driving.left.velocity,     0.5*1*2/params.getLeftWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.right.velocity,    0.5*1*2/params.getRightWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.driving.rear.velocity,     0.5*1*2/params.getRearWheelDiameter(), 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.left.position,    M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.right.position,   M_PI, 1E-6  );
    BOOST_CHECK_CLOSE( motorsCmd.steering.rear.position,    M_PI, 1E-6 );

}



