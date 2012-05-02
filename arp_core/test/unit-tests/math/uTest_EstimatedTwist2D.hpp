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

BOOST_AUTO_TEST_CASE( EstimatedTwist2_Default_Constructor )
{
	// Test default constructor
	EstimatedTwist2D a;
	BOOST_CHECK( a.vx() == 0.0 );
	BOOST_CHECK( a.vy() == 0.0 );
	BOOST_CHECK( a.vh() == 0.0 );
}

BOOST_AUTO_TEST_CASE( EstimatedTwist2_Accessors )
{
	Twist2D a;
	a.vx(1.);
	a.vy(2.);
	a.vh(3.);
	BOOST_CHECK( a.vx() == 1. );
	BOOST_CHECK( a.vy() == 2. );
	BOOST_CHECK( a.vh() == 3. );
}


//BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_trivial )
//{
//    EstimatedTwist2D a(1.3, -2.8, 3.);
//    Eigen::Matrix<double,3,3> cov;
//    cov << 0.5, 0., 0.,
//           0. , 1., 0.,
//           0. , 0., 2.;
//    a.cov( cov );
//    a.date( 100. );
//
//    EstimatedTwist2D res;
//
//    Pose2D x(0,0);
//    res = a.transport( Pose2D(0,0,0) );
//    BOOST_CHECK_EQUAL( res.vx() ,  a.vx() );
//    BOOST_CHECK_EQUAL( res.vy() ,  a.vy() );
//    BOOST_CHECK_EQUAL( res.vh() ,  a.vh() );
//    BOOST_CHECK_EQUAL( res.date(), a.date() );
//    BOOST_CHECK_EQUAL( res.cov(),  a.cov() );
//}


//BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_translation )
//{
//    EstimatedTwist2D a(1.3, -2.8, 0.);
//    Eigen::Matrix<double,3,3> cov;
//    cov << 0.5, 0., 0.,
//           0. , 1., 0.,
//           0. , 0., 2.;
//    a.cov( cov );
//    a.date( 100. );
//
//    EstimatedTwist2D res;
//
//    res = a.transport( Pose2D(1,-2,0) );
//    BOOST_CHECK_EQUAL( res.vx() ,  a.vx() );
//    BOOST_CHECK_EQUAL( res.vy() ,  a.vy() );
//    BOOST_CHECK_EQUAL( res.vh() ,  a.vh() );
//    BOOST_CHECK_EQUAL( res.date(), a.date() );
//    BOOST_CHECK_EQUAL( res.cov(),  a.cov() );
//}

//BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_translation_2 )
//{
//    EstimatedTwist2D a(1.3, -2.8, 0.);
//    Eigen::Matrix<double,3,3> cov;
//    cov << 0.5, 0., 0.,
//           0. , 1., 0.,
//           0. , 0., 2.;
//    a.cov( cov );
//    a.date( 100. );
//
//    EstimatedTwist2D res;
//
//    res = a.transport( Pose2D(1,-2,0) );
//    BOOST_CHECK_EQUAL( res.vx() ,  a.vx() );
//    BOOST_CHECK_EQUAL( res.vy() ,  a.vy() );
//    BOOST_CHECK_EQUAL( res.vh() ,  a.vh() );
//    BOOST_CHECK_EQUAL( res.date(), a.date() );
//    BOOST_CHECK_EQUAL( res.cov(),  a.cov() );
//}
//
//BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_translation_3 )
//{
//    EstimatedTwist2D a(1.3, -2.8, 0.);
//    Eigen::Matrix<double,3,3> cov;
//    cov << 0.5, 0., 0.,
//           0. , 1., 0.,
//           0. , 0., 2.;
//    a.cov( cov );
//    a.date( 100. );
//
//    EstimatedTwist2D res;
//
//    Pose2D demitour(0,0,M_PI);
//    res = a.transport( demitour );
//    BOOST_CHECK_EQUAL( res.vx() ,  a.vx() );
//    BOOST_CHECK_EQUAL( res.vy() ,  a.vy() );
//    BOOST_CHECK_EQUAL( res.vh() ,  a.vh() );
//    BOOST_CHECK_EQUAL( res.date(), a.date() );
//    BOOST_CHECK_EQUAL( res.cov(),  a.cov() );
//
////    Pose2D x(1,0);
////    Pose2D y(0,1);
////    Pose2D demitour(0,0,M_PI);
////    Pose2D quarttour(0,0,M_PI_2);
////
////    BOOST_CHECK( a.transport(x) == a );
////    BOOST_CHECK( a.transport(y) == a );
////
////    res = a.transport(demitour);
////    BOOST_CHECK_CLOSE( res.vx() ,  -a.vx() , 1E-6);
////    BOOST_CHECK_CLOSE( res.vy() ,  -a.vy() , 1E-6);
////    BOOST_CHECK_CLOSE( res.vh() ,  a.vh()  , 1E-6);
////
////    res = a.transport(quarttour);
////    BOOST_CHECK_CLOSE( res.vx() ,  a.vy() , 1E-6);
////    BOOST_CHECK_CLOSE( res.vy() ,  -a.vx(), 1E-6);
////    BOOST_CHECK_CLOSE( res.vh() ,  a.vh() , 1E-6);
//}
//
//BOOST_AUTO_TEST_CASE( EstimatedTwist2_TransportRotation )
//{
//    EstimatedTwist2D a(0,0,2.6);
//    EstimatedTwist2D res;
//    Pose2D demitour(0,0,M_PI);
//    Pose2D quarttour(0,0,M_PI_2);
//    Pose2D x(1,0);
//    Pose2D y(0,1);
//
//    res = a.transport(demitour);
//    BOOST_CHECK_CLOSE( res.vx() ,  0 , 1E-6);
//    BOOST_CHECK_CLOSE( res.vy() ,  0 , 1E-6);
//    BOOST_CHECK_CLOSE( res.vh() ,  2.6  , 1E-6);
//
//    res = a.transport(quarttour);
//    BOOST_CHECK_CLOSE( res.vx() ,  0, 1E-6);
//    BOOST_CHECK_CLOSE( res.vy() ,  0, 1E-6);
//    BOOST_CHECK_CLOSE( res.vh() ,  2.6, 1E-6);
//
//    res = a.transport(x);
//    //cerr << x.toString() << " -> " << x.getBigAdjoint() << endl;
//    //cerr << a.toString() << " -> " << a.transport(x).toString() << endl;
//    //cerr << a.toString() << " -> " << a.transport(y).toString() << endl;
//
//    BOOST_CHECK_CLOSE( res.vx() ,  0, 1E-6);
//    BOOST_CHECK_CLOSE( res.vy() ,  2.6, 1E-6);
//    BOOST_CHECK_CLOSE( res.vh() ,  2.6, 1E-6);
//
//    res = a.transport(y);
//    BOOST_CHECK_CLOSE( res.vx() ,  -2.6, 1E-6);
//    BOOST_CHECK_CLOSE( res.vy() ,  0, 1E-6);
//    BOOST_CHECK_CLOSE( res.vh() ,  2.6, 1E-6);
//}
//
