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


BOOST_AUTO_TEST_CASE( UbiquityKinematics_checkTest )
{
    arp_model::UbiquityParams params;
    BOOST_CHECK_EQUAL( params.check() , true);
}

BOOST_AUTO_TEST_CASE( UbiquityKinematics_leftWheelDiameter )
{
    arp_model::UbiquityParams params;
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
    arp_model::UbiquityParams params;
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
    arp_model::UbiquityParams params;
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
