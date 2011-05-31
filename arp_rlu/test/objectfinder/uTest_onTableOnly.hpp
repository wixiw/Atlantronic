/*
 * uTest_ComputeSegment.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "objectfinder/ObjectFinder.hpp"
#include "math/math.hpp"

using namespace Eigen;
using namespace arp_rlu;
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( onTableOnly_Test_1 )
{
    ObjectFinder objf;
    Scan cart = MatrixXd::Zero(4, 9);
    cart <<  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
             0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            -1.4, -0.2,  1.5, -1.4, -0.2,  1.5, -1.4, -0.2,  1.5,
             1.2,  1.2,  1.2,  0.3,  0.3,  0.3, -0.9, -0.9, -0.9;
    objf.setCartesianScan(cart);
    Scan s = objf.onTableOnly();
    BOOST_CHECK_EQUAL( objf.xMinTable, -1.3 );
    BOOST_CHECK_EQUAL( objf.xMaxTable,  1.3 );
    BOOST_CHECK_EQUAL( objf.yMinTable, -0.85 );
    BOOST_CHECK_EQUAL( objf.yMaxTable,  1.0 );
    BOOST_CHECK_EQUAL( s.rows(), 4 );
    BOOST_CHECK_EQUAL( s.cols(), 1 );
    BOOST_CHECK_CLOSE( -0.2, s(2,0), 1.f);
    BOOST_CHECK_CLOSE(  0.3, s(3,0), 1.f);
}
