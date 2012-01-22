/*
 * uTest_CartesianCrop.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/CartesianCrop.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_CartesianCrop )

BOOST_AUTO_TEST_CASE( Test_1 )
{
    lsl::LaserScan rawScan;
    lsl::LaserScan filtScan = lsl::CartesianCrop::apply(rawScan);
}

BOOST_AUTO_TEST_SUITE_END()
