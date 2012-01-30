/*
 * uTest_CircleIdentif.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/CircleIdentif.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_CircleIdentif )

BOOST_AUTO_TEST_CASE( Test_1 )
{
    lsl::DetectedObject rawObject;
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject);
}

BOOST_AUTO_TEST_SUITE_END()
