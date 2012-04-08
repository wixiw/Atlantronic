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
    arp_model::AxesGroup steeringMotorVelocities;
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
