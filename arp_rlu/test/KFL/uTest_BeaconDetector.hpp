/*
 * uTest_BeaconDetector.hpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "KFL/BeaconDetector.hpp"

#include "tools/vjson/JsonDocument.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;


BOOST_AUTO_TEST_SUITE( unittest_BeaconDetector )

BOOST_AUTO_TEST_CASE( Constructor_1 )
{
//    kfl::BeaconDetector obj;

    vjson::JsonDocument doc;
    if( doc.parse("./test.json") )
    {
        std::cout << "parsing OK" << std::endl;
    }

}

BOOST_AUTO_TEST_SUITE_END()
