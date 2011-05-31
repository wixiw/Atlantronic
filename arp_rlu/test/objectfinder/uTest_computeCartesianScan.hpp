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

BOOST_AUTO_TEST_CASE( computeCartesianScan_Test_1 )
{
    ObjectFinder objf;
    Scan polar = MatrixXd::Zero(0, 0);
    objf.setPolarScan(polar);
    double xOnTable = 0.;
    double yOnTable = 0.;
    double thetaOnTable = 0.;
    Scan cart = objf.computeCartesianScan(xOnTable, yOnTable, thetaOnTable);
    BOOST_CHECK_EQUAL( cart.rows(), 0 );
    BOOST_CHECK_EQUAL( cart.cols(), polar.cols() );
}

BOOST_AUTO_TEST_CASE( computeCartesianScan_Test_2 )
{
    ObjectFinder objf;
    Scan polar = MatrixXd::Zero(2, 4);
    polar << 0., PI/2., PI, -PI/2.,
    1.0, 0.5, 2.0, 3.0;
    objf.setPolarScan(polar);
    double xOnTable = 0.;
    double yOnTable = 0.;
    double thetaOnTable = 0.;
    Scan cart = objf.computeCartesianScan(xOnTable, yOnTable, thetaOnTable);

    BOOST_CHECK_EQUAL( cart.rows(), 4 );
    BOOST_CHECK_EQUAL( cart.cols(), polar.cols() );
    for(unsigned int i = 0; i < polar.cols(); i++)
    {
        BOOST_CHECK_EQUAL( cart(0,i), polar(0,i) );
        BOOST_CHECK_EQUAL( cart(1,i), polar(1,i) );
    }
    BOOST_CHECK_CLOSE( 1.0, cart(2,0), 1.f); BOOST_CHECK( abs(cart(3,0)) < 0.00001f);
    BOOST_CHECK( abs(cart(2,1)) < 0.00001f); BOOST_CHECK_CLOSE( 0.5, cart(3,1), 1.f);
    BOOST_CHECK_CLOSE(-2.0, cart(2,2), 1.f); BOOST_CHECK( abs(cart(3,2)) < 0.00001f);
    BOOST_CHECK( abs(cart(2,3)) < 0.00001f); BOOST_CHECK_CLOSE(-3.0, cart(3,3), 1.1f);
}

BOOST_AUTO_TEST_CASE( computeCartesianScan_Test_3 )
{
    ObjectFinder objf;
    Scan polar = MatrixXd::Zero(2, 4);
    polar << 0., PI/2., PI, -PI/2.,
    1.0, 0.5, 2.0, 3.0;
    objf.setPolarScan(polar);
    double xOnTable = 0.3;
    double yOnTable = -0.1;
    double thetaOnTable = 0.;
    Scan cart = objf.computeCartesianScan(xOnTable, yOnTable, thetaOnTable);

    BOOST_CHECK_EQUAL( cart.rows(), 4 );
    BOOST_CHECK_EQUAL( cart.cols(), polar.cols() );
    for(unsigned int i = 0; i < polar.cols(); i++)
    {
        BOOST_CHECK_EQUAL( cart(0,i), polar(0,i) );
        BOOST_CHECK_EQUAL( cart(1,i), polar(1,i) );
    }
    BOOST_CHECK_CLOSE( 1.3, cart(2,0), 1.f); BOOST_CHECK_CLOSE(-0.1, cart(3,0), 1.f);
    BOOST_CHECK_CLOSE( 0.3, cart(2,1), 1.f); BOOST_CHECK_CLOSE( 0.4, cart(3,1), 1.f);
    BOOST_CHECK_CLOSE(-1.7, cart(2,2), 1.f); BOOST_CHECK_CLOSE(-0.1, cart(3,2), 1.f);
    BOOST_CHECK_CLOSE( 0.3, cart(2,3), 1.f); BOOST_CHECK_CLOSE(-3.1, cart(3,3), 1.f);
}

BOOST_AUTO_TEST_CASE( computeCartesianScan_Test_4 )
{
    ObjectFinder objf;
    Scan polar = MatrixXd::Zero(2, 4);
    polar << 0., PI/2., PI, -PI/2.,
    1.0, 0.5, 2.0, 3.0;
    objf.setPolarScan(polar);
    double xOnTable = 0.3;
    double yOnTable = -0.1;
    double thetaOnTable = PI/2.;
    Scan cart = objf.computeCartesianScan(xOnTable, yOnTable, thetaOnTable);

    BOOST_CHECK_EQUAL( cart.rows(), 4 );
    BOOST_CHECK_EQUAL( cart.cols(), polar.cols() );
    for(unsigned int i = 0; i < polar.cols(); i++)
    {
        BOOST_CHECK_EQUAL( cart(0,i), polar(0,i) );
        BOOST_CHECK_EQUAL( cart(1,i), polar(1,i) );
    }
    BOOST_CHECK_CLOSE( 0.3, cart(2,0), 1.f); BOOST_CHECK_CLOSE( 0.9, cart(3,0), 1.f);
    BOOST_CHECK_CLOSE(-0.2, cart(2,1), 1.f); BOOST_CHECK_CLOSE(-0.1, cart(3,1), 1.f);
    BOOST_CHECK_CLOSE( 0.3, cart(2,2), 1.f); BOOST_CHECK_CLOSE(-2.1, cart(3,2), 1.f);
    BOOST_CHECK_CLOSE( 3.3, cart(2,3), 1.f); BOOST_CHECK_CLOSE(-0.1, cart(3,3), 1.f);
}
