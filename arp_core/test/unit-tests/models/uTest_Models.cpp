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
    //  arp_rlu::kfl::Logger::InitNull("uT_KFL", DEBUG);
    arp_model::Logger::InitConsole("uT_Models", DEBUG);
//      arp_rlu::kfl::Logger::InitFile("uT_KFL", DEBUG);

//      arp_rlu::kfl::Log( FATAL )  << "fatal";
//      arp_rlu::kfl::Log( EMERG )  << "emerg";
//      arp_rlu::kfl::Log( ALERT )  << "alert";
//      arp_rlu::kfl::Log( CRIT )   << "crit";
//      arp_rlu::kfl::Log( ERROR )  << "error";
//      arp_rlu::kfl::Log( WARN )   << "warn";
//      arp_rlu::kfl::Log( NOTICE ) << "notice";
//      arp_rlu::kfl::Log( INFO )   << "info";
//      arp_rlu::kfl::Log( DEBUG )  << "debug";

    //  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_successful_tests );
    //  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_messages );
    //  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_warnings );
    //  boost::unit_test::unit_test_log.set_threshold_level( boost::unit_test::log_all_errors );

    //  boost::unit_test::unit_test_log.set_stream( std::cout );

}
BOOST_AUTO_TEST_SUITE_END()

#include "uTest_UbiquityParams.hpp"
#include "uTest_UK_MotorTurret.hpp"
#include "uTest_UK_Twist2Turrets.hpp"
//#include "uTest_UK_Turrets2Twist.hpp"
