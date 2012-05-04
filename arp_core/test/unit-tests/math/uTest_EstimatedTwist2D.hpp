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


BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_trivial )
{
    Eigen::Matrix<double,3,3> cov;
    cov << 0.5, 0., 0.,
           0. , 1., 0.,
           0. , 0., 2.;
    EstimatedTwist2D a = MathFactory::createEstimatedTwist2DFromCartesianRepr(1.3, -2.8, 3., 100., cov);

    EstimatedTwist2D res;

    res = a.transport( Pose2D(0,0,0) );
    BOOST_CHECK_EQUAL( res.vx() ,  a.vx() );
    BOOST_CHECK_EQUAL( res.vy() ,  a.vy() );
    BOOST_CHECK_EQUAL( res.vh() ,  a.vh() );
    BOOST_CHECK_EQUAL( res.date(), a.date() );
    BOOST_CHECK_EQUAL( res.cov(),  a.cov() );
}


BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_Rotation_1 )
{
    Eigen::Matrix<double,3,3> cov;
    cov << 5., 0., 0.,
           0., 3., 0.,
           0., 0., 2.;

    Vector3 T(1., 2., 4.);
    Pose2D p(0.,0.,M_PI);

    EstimatedTwist2D a = MathFactory::createEstimatedTwist2DFromCartesianRepr(T, 100., cov);
    Twist2D b = (Twist2D) a;
    Twist2D ground = b.transport( p );

    EstimatedTwist2D res;

    res = a.transport( p );
    BOOST_CHECK_EQUAL( res.vx() ,  ground.vx() );
    BOOST_CHECK_EQUAL( res.vy() ,  ground.vy() );
    BOOST_CHECK_EQUAL( res.vh() ,  ground.vh() );
    BOOST_CHECK_EQUAL( res.date(), a.date() );
    Covariance3 covRes = res.cov();
    BOOST_CHECK_EQUAL( covRes(0,0), cov(0,0) );
    BOOST_CHECK_EQUAL( covRes(1,1), cov(1,1) );
    BOOST_CHECK_EQUAL( covRes(2,2), cov(2,2) );
}

BOOST_AUTO_TEST_CASE( EstimatedTwist2_Transport_Rotation_2 )
{
    Eigen::Matrix<double,3,3> cov;
    cov << 5., 0., 0.,
            0., 3., 0.,
            0., 0., 2.;

    Vector3 T(1., 2., 4.);
    Pose2D p(0.,0.,M_PI/2);

    EstimatedTwist2D a = MathFactory::createEstimatedTwist2DFromCartesianRepr(T, 100., cov);
    Twist2D b = (Twist2D) a;
    Twist2D ground = b.transport( p );

    EstimatedTwist2D res;

    res = a.transport( p );
    BOOST_CHECK_EQUAL( res.vx() ,  ground.vx() );
    BOOST_CHECK_EQUAL( res.vy() ,  ground.vy() );
    BOOST_CHECK_EQUAL( res.vh() ,  ground.vh() );
    BOOST_CHECK_EQUAL( res.date(), a.date() );
    Covariance3 covRes = res.cov();
    BOOST_CHECK_EQUAL( covRes(0,0), cov(0,0) );
    BOOST_CHECK_EQUAL( covRes(1,1), cov(2,2) );
    BOOST_CHECK_EQUAL( covRes(2,2), cov(1,1) );
}

