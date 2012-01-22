/*
 * uTest_KFLocalizator.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "KFL/KFLocalizator.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_KFLocalizator )

BOOST_AUTO_TEST_CASE( Constructor_1 )
{
    kfl::KFLocalizator obj;
}

BOOST_AUTO_TEST_SUITE_END()
