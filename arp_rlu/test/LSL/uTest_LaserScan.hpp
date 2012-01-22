/*
 * uTest_LaserScan.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/LaserScan.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_LaserScan )

BOOST_AUTO_TEST_CASE( Constructor_1 )
{
    lsl::LaserScan obj;
}

BOOST_AUTO_TEST_SUITE_END()
