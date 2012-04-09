/*
 * uTest_LSL.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#include <ros/package.h>

#include "LSL/Logger.hpp"

using namespace arp_core::log;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE uTest_LSL
#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>

BOOST_AUTO_TEST_SUITE( unittest_dummy )
BOOST_AUTO_TEST_CASE( testdummy )
{
//  arp_rlu::lsl::Logger::InitNull("uT_LSL", DEBUG);
  arp_rlu::lsl::Logger::InitConsole("uT_LSL", ERROR);
//  arp_rlu::lsl::Logger::InitConsole("uT_LSL", DEBUG);
//  arp_rlu::lsl::Logger::InitFile("uT_LSL", DEBUG);

//  arp_rlu::lsl::Log( FATAL )  << "fatal";
//  arp_rlu::lsl::Log( EMERG )  << "emerg";
//  arp_rlu::lsl::Log( ALERT )  << "alert";
//  arp_rlu::lsl::Log( CRIT )   << "crit";
//  arp_rlu::lsl::Log( ERROR )  << "error";
//  arp_rlu::lsl::Log( WARN )   << "warn";
//  arp_rlu::lsl::Log( NOTICE ) << "notice";
//  arp_rlu::lsl::Log( INFO )   << "info";
//  arp_rlu::lsl::Log( DEBUG )  << "debug";

//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_successful_tests );
//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_messages );
//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_warnings );
//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_all_errors );

//  boost::unit_test::unit_test_log.set_stream( std::cout );

}
BOOST_AUTO_TEST_SUITE_END()

#include "uTest_LaserScan.hpp"

#include "objects/uTest_DetectedObject.hpp"
#include "objects/uTest_Circle.hpp"
#include "objects/uTest_DetectedCircle.hpp"

#include "filters/uTest_MedianFilter.hpp"
#include "filters/uTest_PolarCrop.hpp"
#include "filters/uTest_CartesianCrop.hpp"
#include "filters/uTest_PolarSegment.hpp"
#include "filters/uTest_CartesianSegment.hpp"
#include "filters/uTest_CircleIdentif.hpp"

#include "identificators/uTest_SoloCircleIdentif.hpp"
#include "identificators/uTest_DuoCircleIdentif.hpp"
#include "identificators/uTest_TrioCircleIdentif.hpp"

#include "tools/uTest_JsonScanParser.hpp"

