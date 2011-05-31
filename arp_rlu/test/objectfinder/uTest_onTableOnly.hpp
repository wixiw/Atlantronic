/*
 * uTest_ComputeSegment.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */


#include "objectfinder/ObjectFinder.hpp"

using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_CASE( ObjectFinder_Test_1 )
{
    ObjectFinder of;
    BOOST_CHECK_EQUAL( 0., 0. );
}
