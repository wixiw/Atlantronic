#define BOOST_TEST_DYN_LINK

#include <boost/test/included/unit_test.hpp>
#include <boost/bind.hpp>
using namespace boost;
using namespace boost::unit_test;
using namespace boost::unit_test::framework;

#include <ARPMain.hpp>
#include <taskcontexts/ARDTaskContext.hpp>
#include <rtt/extras/SimulationActivity.hpp>

using namespace arp_core;
using namespace std;
using namespace RTT;
using namespace RTT::detail;
using namespace RTT::extras;
using namespace OCL;


void testComponentConfiguration()
{
    ARDTaskContext component("Bite");
    component.setActivity(new SimulationActivity(0.01));

    BOOST_CHECK( component.configure() );
    BOOST_CHECK( component.isConfigured() );
    BOOST_CHECK( !component.isRunning() );

    BOOST_CHECK( component.start() );
    BOOST_CHECK( component.isConfigured() );
    BOOST_CHECK( component.isRunning() );

    component.stop();
    BOOST_CHECK( !component.isRunning() );
    BOOST_CHECK( component.isConfigured() );

    component.cleanup();
    BOOST_CHECK( !component.isRunning() );
    BOOST_CHECK( !component.isConfigured() );

}



bool
init_function()
{
    /** nom de la suite de test principale */
    master_test_suite().p_name.value = "ARDTaskContext Test Suite";

    master_test_suite().add( BOOST_TEST_CASE( ::bind( &testComponentConfiguration) ) );

    return true;
}

int ARP_main(int argc, char** argv)
{
    unit_test_main( &init_function, argc, argv );
    return EXIT_SUCCESS;
}
