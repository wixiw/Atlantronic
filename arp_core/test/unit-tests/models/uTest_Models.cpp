/*
 * uTest_Models.cpp
 *
 *  Created on: 4 April 2012
 *      Author: ard
 */

#include "models/Logger.hpp"

using namespace arp_core::log;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE uTest_Models
#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>

BOOST_AUTO_TEST_SUITE( unittest_dummy )
BOOST_AUTO_TEST_CASE( testdummy )
{
    //arp_model::Logger::InitNull("uT_KFL", DEBUG);
    arp_model::Logger::InitConsole("uT_Models", INFO);
    //arp_model::Logger::InitFile("uT_KFL", DEBUG);

}
BOOST_AUTO_TEST_SUITE_END()

#include "uTest_UbiquityParams.hpp"
#include "uTest_UK_MotorTurret.hpp"
#include "uTest_UK_Twist2Turrets.hpp"
#include "uTest_UK_Turrets2Twist.hpp"
#include "uTest_UbiquityKinematics.hpp"
