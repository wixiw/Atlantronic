/*
 * uTest_KFL.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <ros/package.h>

#include "KFL/Logger.hpp"
#include "LSL/Logger.hpp"

using namespace arp_core::log;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE uTest_KFL
#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>

BOOST_AUTO_TEST_SUITE( unittest_dummy )
BOOST_AUTO_TEST_CASE( testdummy )
{
    //  arp_rlu::kfl::Logger::InitNull("uT_KFL", DEBUG);
    arp_rlu::kfl::Logger::InitConsole("uT_KFL", NOTICE);
//      arp_rlu::kfl::Logger::InitFile("uT_KFL", DEBUG);

    arp_rlu::lsl::Logger::InitConsole("uT_LSL", NOTICE);

}
BOOST_AUTO_TEST_SUITE_END()

#include "uTest_BeaconDetector.hpp"

#include "BFL/uTest_BFLWrapper.hpp"

#include "uTest_KFLocalizator_Trivial.hpp"
#include "uTest_KFLocalizator_Static.hpp"
#include "uTest_KFLocalizator_Dynamic.hpp"

