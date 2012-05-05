/*
 * test_EstimatedTwist2.cpp
 *
 *  Created on: 13 sept. 2010
 *      Author: boris
 */

#include <math/core>
#include <iostream>
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( EstimatedPose2_Default_Constructor )
{
    // Test default constructor
    EstimatedPose2D a;
    BOOST_CHECK( a.x() == 0.0 );
    BOOST_CHECK( a.y() == 0.0 );
    BOOST_CHECK( a.h() == 0.0 );
}

BOOST_AUTO_TEST_CASE( EstimatedPose2_Accessors )
{
    EstimatedPose2D a;
    a.x(1.);
    a.y(2.);
    a.h(3.);
    a.date( 100. );
    Covariance3 c = Covariance3::Random();
    c = ( c + c.transpose() ) * 0.5;
    a.cov( c );
    BOOST_CHECK( a.x() == 1. );
    BOOST_CHECK( a.y() == 2. );
    BOOST_CHECK( a.h() == 3. );
    BOOST_CHECK( a.date() == 100. );
    BOOST_CHECK( a.cov() == c );
}



BOOST_AUTO_TEST_CASE( EstimatedPose2_Operator_trivial )
{
    srand( time(NULL) );
    double x = rand() / (double)RAND_MAX * 20 - 10.;
    double y = rand() / (double)RAND_MAX * 20 - 10.;
    double h = rand() / (double)RAND_MAX * 20 - 10.;
    double date = rand() / (double)RAND_MAX * 20;
    Covariance3 cov = Covariance3::Random();
    cov = ( cov + cov.transpose() ) * 0.5;

    EstimatedPose2D a;
    a.x( );
    a.y(2.);
    a.h(3.);
    a.date( 100. );
    a.cov( cov );
    Pose2D p;
    BOOST_CHECK( a.x() == 1. );
    BOOST_CHECK( a.y() == 2. );
    BOOST_CHECK( a.h() == 3. );
    BOOST_CHECK( a.date() == 100. );
    BOOST_CHECK( a.cov() == cov );
}

