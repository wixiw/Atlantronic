/*
 * uTest_ODS.cpp
 *
 *  Created on: 08 April 2012
 *      Author: willy
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE uTest_LSL

#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>
#include <models/Logger.hpp>

using namespace arp_core::log;

BOOST_AUTO_TEST_SUITE( unittest_dummy )
BOOST_AUTO_TEST_CASE( testdummy )
{
    //arp_model::model::Logger::InitNull("uT_KFL", DEBUG);
    arp_model::Logger::InitConsole("uT_ODS", INFO);
    //arp_model::model::Logger::InitFile("uT_KFL", DEBUG);

}
BOOST_AUTO_TEST_SUITE_END()

#include "uTest_KinematicFilter.hpp"
