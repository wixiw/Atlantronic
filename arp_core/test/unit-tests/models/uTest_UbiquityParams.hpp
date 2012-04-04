/*
 * uTest_UbiquityParams.hpp
 *
 *  Created on: 04 April 2012
 *      Author: willy
 *
 *      Teste les paramètres du robot
 */

#include <boost/test/floating_point_comparison.hpp>
#include "models/UbiquityParams.hpp"

using namespace arp_math;
using namespace arp_core;

BOOST_AUTO_TEST_CASE( UbiquityKinematics_checkTest )
{
    UbiquityParams params;
    BOOST_CHECK_EQUAL( params.check() , true);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_leftWheelDiameter )
{
    UbiquityParams params;
    //null
    params.setLeftWheelDiameter(0.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    //negatif
    params.setRearWheelDiameter(-1.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    //trop grand
    params.setRearWheelDiameter(3.0);
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_rightWheelDiameter )
{
    UbiquityParams params;
    //null
    params.setRightWheelDiameter(0.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    //negatif
    params.setRearWheelDiameter(-1.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    //trop grand
    params.setRearWheelDiameter(3.0);
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_rearWheelDiameter )
{
    UbiquityParams params;
    //null
    params.setRearWheelDiameter(0.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    //negatif
    params.setRearWheelDiameter(-1.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    //trop grand
    params.setRearWheelDiameter(3.0);
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_turretRatio )
{
    UbiquityParams params;
    params.setTurretRatio(0.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    params.setTurretRatio(-10.0);
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_tractionRatio )
{
    UbiquityParams params;
    params.setTractionRatio(0.0);
    BOOST_CHECK_EQUAL( params.check() , false);
    params.setTractionRatio(-10.0);
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_leftRightTurret )
{
    UbiquityParams params;
    params.setLeftTurretPosition(params.getRightTurretPosition());
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_leftRearTurret )
{
    UbiquityParams params;
    params.setLeftTurretPosition(params.getRearTurretPosition());
    BOOST_CHECK_EQUAL( params.check() , false);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_rightRearTurret )
{
    UbiquityParams params;
    params.setRightTurretPosition(params.getRearTurretPosition());
    BOOST_CHECK_EQUAL( params.check() , false);
}
