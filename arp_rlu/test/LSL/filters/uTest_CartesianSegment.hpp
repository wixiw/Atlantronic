/*
 * uTest_CartesianSegment.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/CartesianSegment.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_CartesianSegment )

BOOST_AUTO_TEST_CASE( Test_1 )
{
    lsl::LaserScan rawScan;
    std::vector<lsl::LaserScan> objects = lsl::CartesianSegment::apply(rawScan);
}

BOOST_AUTO_TEST_SUITE_END()
