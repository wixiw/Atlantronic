/*
 * uTest_MedianFilter.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/MedianFilter.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_MedianFilter )

BOOST_AUTO_TEST_CASE( Test_1 )
{
    lsl::LaserScan rawScan;
    lsl::LaserScan filtScan = lsl::MedianFilter::apply(rawScan);
}

BOOST_AUTO_TEST_SUITE_END()
