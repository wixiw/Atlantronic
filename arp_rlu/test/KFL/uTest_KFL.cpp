/*
 * uTest_KFL.cpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */


#include "KFL/Logger.hpp"

using namespace arp_core::log;

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE uTest_KFL
#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>

BOOST_AUTO_TEST_SUITE( unittest_dummy )
BOOST_AUTO_TEST_CASE( testdummy )
{
//  arp_rlu::kfl::Logger::InitNull("uT_KFL", DEBUG);
  arp_rlu::kfl::Logger::InitConsole("uT_KFL", DEBUG);
//  arp_rlu::kfl::Logger::InitFile("uT_KFL", DEBUG);

//  arp_rlu::kfl::Log( FATAL )  << "fatal";
//  arp_rlu::kfl::Log( EMERG )  << "emerg";
//  arp_rlu::kfl::Log( ALERT )  << "alert";
//  arp_rlu::kfl::Log( CRIT )   << "crit";
//  arp_rlu::kfl::Log( ERROR )  << "error";
//  arp_rlu::kfl::Log( WARN )   << "warn";
//  arp_rlu::kfl::Log( NOTICE ) << "notice";
//  arp_rlu::kfl::Log( INFO )   << "info";
//  arp_rlu::kfl::Log( DEBUG )  << "debug";

//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_successful_tests );
//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_messages );
//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_warnings );
//  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_all_errors );

//  boost::unit_test::unit_test_log.set_stream( std::cout );

}
BOOST_AUTO_TEST_SUITE_END()

//#include "uTest_BeaconDetector.hpp"

#include "BFL/uTest_BFLWrapper.hpp"
//
//#include "uTest_KFLocalizator.hpp"

