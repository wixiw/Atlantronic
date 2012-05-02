/*
 * test_Twist2.cpp
 *
 *  Created on: 13 sept. 2010
 *      Author: boris
 */

#include <math/core>
#include <iostream>
using namespace arp_math;
using namespace std;

BOOST_AUTO_TEST_CASE( MathFactory_Pose2D )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double x = rand() / (double)RAND_MAX * 20 - 10.;
        double y = rand() / (double)RAND_MAX * 20 - 10.;
        double h = rand() / (double)RAND_MAX * 20 - 10.;
        Pose2D p = MathFactory::createPose2D(Vector2(x,y), Rotation2(h));
        BOOST_CHECK_EQUAL( p.x() , x );
        BOOST_CHECK_EQUAL( p.y() , y );
        BOOST_CHECK_EQUAL( p.h() , h );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_EstimatedPose2D_1 )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double x = rand() / (double)RAND_MAX * 20 - 10.;
        double y = rand() / (double)RAND_MAX * 20 - 10.;
        double h = rand() / (double)RAND_MAX * 20 - 10.;
        double date = rand() / (double)RAND_MAX * 20;
        Covariance3 cov = Covariance3::Random();
        cov = (cov + cov.transpose()) * 0.5;
        EstimatedPose2D p = MathFactory::createEstimatedPose2D(Vector2(x,y), Rotation2(h), date, cov);
        BOOST_CHECK_EQUAL( p.x() , x );
        BOOST_CHECK_EQUAL( p.y() , y );
        BOOST_CHECK_EQUAL( p.h() , h );
        BOOST_CHECK_EQUAL( p.date() , date );
        BOOST_CHECK_EQUAL( p.cov() , cov );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_EstimatedPose2D_2 )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double x = rand() / (double)RAND_MAX * 20 - 10.;
        double y = rand() / (double)RAND_MAX * 20 - 10.;
        double h = rand() / (double)RAND_MAX * 20 - 10.;
        double date = rand() / (double)RAND_MAX * 20;
        Covariance3 cov = Covariance3::Random();
        cov = (cov + cov.transpose()) * 0.5;
        EstimatedPose2D p = MathFactory::createEstimatedPose2D(x, y, h, date, cov);
        BOOST_CHECK_EQUAL( p.x() , x );
        BOOST_CHECK_EQUAL( p.y() , y );
        BOOST_CHECK_EQUAL( p.h() , h );
        BOOST_CHECK_EQUAL( p.date() , date );
        BOOST_CHECK_EQUAL( p.cov() , cov );
    }
}


BOOST_AUTO_TEST_CASE( MathFactory_Twist2D_CartesianRepr_triplet_double )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double vx = rand() / (double)RAND_MAX * 20 - 10.;
        double vy = rand() / (double)RAND_MAX * 20 - 10.;
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        Twist2D t = MathFactory::createTwist2DFromCartesianRepr(vx,vy,vh);
        BOOST_CHECK_EQUAL( t.vx() , vx );
        BOOST_CHECK_EQUAL( t.vy() , vy );
        BOOST_CHECK_EQUAL( t.vh() , vh );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_Twist2D_CartesianRepr_Vector2_double )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double vx = rand() / (double)RAND_MAX * 20 - 10.;
        double vy = rand() / (double)RAND_MAX * 20 - 10.;
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        Twist2D t = MathFactory::createTwist2DFromCartesianRepr(arp_math::Vector2(vx,vy), vh);
        BOOST_CHECK_EQUAL( t.vx() , vx );
        BOOST_CHECK_EQUAL( t.vy() , vy );
        BOOST_CHECK_EQUAL( t.vh() , vh );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_Twist2D_CartesianRepr_Vector3 )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double vx = rand() / (double)RAND_MAX * 20 - 10.;
        double vy = rand() / (double)RAND_MAX * 20 - 10.;
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        Twist2D t = MathFactory::createTwist2DFromCartesianRepr(arp_math::Vector3(vh,vx,vy));
        BOOST_CHECK_EQUAL( t.vx() , vx );
        BOOST_CHECK_EQUAL( t.vy() , vy );
        BOOST_CHECK_EQUAL( t.vh() , vh );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_Twist2D_CreateFromPolarRepr )
{
    Twist2D twistCart,twistPolar;

    twistPolar = MathFactory::createTwist2DFromPolarRepr(0,0,0);
    twistCart = Twist2D();
    BOOST_CHECK( twistPolar == twistCart );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(0,0,1);
    twistCart = Twist2D(0,0,1);
    BOOST_CHECK( twistPolar == twistCart );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(0,1,0);
    twistCart = Twist2D(0,0,0);
    BOOST_CHECK( twistPolar == twistCart );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(0,1,0);
    twistCart = Twist2D(0,0,0);
    BOOST_CHECK( twistPolar == twistCart );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(1,0,0);
    twistCart = Twist2D(1,0,0);
    BOOST_CHECK( twistPolar == twistCart );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(1,M_PI,0);
    twistCart = Twist2D(-1,0,0);
    BOOST_CHECK_CLOSE( twistPolar.vx(), twistCart.vx(), 1E-6 );
    BOOST_CHECK_SMALL( twistPolar.vy(), 1E-6 );
    BOOST_CHECK_SMALL( twistPolar.vh(), 1E-6 );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(1,M_PI/2,0);
    twistCart = Twist2D(0,1,0);
    BOOST_CHECK_SMALL( twistPolar.vx(), 1E-6 );
    BOOST_CHECK_CLOSE( twistPolar.vy(), twistCart.vy(), 1E-6 );
    BOOST_CHECK_SMALL( twistPolar.vh(), 1E-6 );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(1,-M_PI/2,0);
    twistCart = Twist2D(0,-1,0);
    BOOST_CHECK_SMALL( twistPolar.vx(), 1E-6 );
    BOOST_CHECK_CLOSE( twistPolar.vy(), twistCart.vy(), 1E-6 );
    BOOST_CHECK_SMALL( twistPolar.vh(), 1E-6 );

    twistPolar = MathFactory::createTwist2DFromPolarRepr(1,M_PI/4,2.7);
    twistCart = Twist2D(sqrt(2)/2,sqrt(2)/2,2.7);
    BOOST_CHECK_CLOSE( twistPolar.vx(), twistCart.vx(), 1E-6 );
    BOOST_CHECK_CLOSE( twistPolar.vy(), twistCart.vy(), 1E-6 );
    BOOST_CHECK_CLOSE( twistPolar.vh(), twistCart.vh(), 1E-6 );
}

BOOST_AUTO_TEST_CASE( MathFactory_EstimatedTwist2D_CartesianRepr_triplet_double )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double vx = rand() / (double)RAND_MAX * 20 - 10.;
        double vy = rand() / (double)RAND_MAX * 20 - 10.;
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        double date = rand() / (double)RAND_MAX * 20;
        Covariance3 cov = Covariance3::Random();
        cov = (cov + cov.transpose()) * 0.5;
        EstimatedTwist2D t = MathFactory::createEstimatedTwist2DFromCartesianRepr(vx,vy,vh, date, cov);
        BOOST_CHECK_EQUAL( t.vx() , vx );
        BOOST_CHECK_EQUAL( t.vy() , vy );
        BOOST_CHECK_EQUAL( t.vh() , vh );
        BOOST_CHECK_EQUAL( t.date() , date );
        BOOST_CHECK_EQUAL( t.cov() , cov );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_EstimatedTwist2D_CartesianRepr_Vector2_double )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double vx = rand() / (double)RAND_MAX * 20 - 10.;
        double vy = rand() / (double)RAND_MAX * 20 - 10.;
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        double date = rand() / (double)RAND_MAX * 20;
        Covariance3 cov = Covariance3::Random();
        cov = (cov + cov.transpose()) * 0.5;
        EstimatedTwist2D t = MathFactory::createEstimatedTwist2DFromCartesianRepr(arp_math::Vector2(vx,vy), vh, date, cov);
        BOOST_CHECK_EQUAL( t.vx() , vx );
        BOOST_CHECK_EQUAL( t.vy() , vy );
        BOOST_CHECK_EQUAL( t.vh() , vh );
        BOOST_CHECK_EQUAL( t.date() , date );
        BOOST_CHECK_EQUAL( t.cov() , cov );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_EstimatedTwist2D_CartesianRepr_Vector3 )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double vx = rand() / (double)RAND_MAX * 20 - 10.;
        double vy = rand() / (double)RAND_MAX * 20 - 10.;
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        double date = rand() / (double)RAND_MAX * 20;
        Covariance3 cov = Covariance3::Random();
        cov = (cov + cov.transpose()) * 0.5;
        EstimatedTwist2D t = MathFactory::createEstimatedTwist2DFromCartesianRepr(arp_math::Vector3(vh,vx,vy), date, cov);
        BOOST_CHECK_EQUAL( t.vx() , vx );
        BOOST_CHECK_EQUAL( t.vy() , vy );
        BOOST_CHECK_EQUAL( t.vh() , vh );
        BOOST_CHECK_EQUAL( t.date() , date );
        BOOST_CHECK_EQUAL( t.cov() , cov );
    }
}

BOOST_AUTO_TEST_CASE( MathFactory_EstimatedTwist2D_CreateFromPolarRepr )
{
    srand( time(NULL) );
    for(unsigned int i = 0 ; i < 10 ; i++)
    {
        double normV = rand() / (double)RAND_MAX * 20;
        double angV = betweenMinusPiAndPlusPi(rand() / (double)RAND_MAX * 20 - 10.);
        double vh = rand() / (double)RAND_MAX * 20 - 10.;
        double date = rand() / (double)RAND_MAX * 20;
        Covariance3 cov = Covariance3::Random();
        cov = (cov + cov.transpose()) * 0.5;
        EstimatedTwist2D twistPolar = MathFactory::createEstimatedTwist2DFromPolarRepr(normV, angV, vh, date, cov);
        EstimatedTwist2D twistGround = EstimatedTwist2D( MathFactory::createTwist2DFromPolarRepr(normV, angV, vh) );
        BOOST_CHECK_EQUAL( twistPolar.vx() , twistGround.vx() );
        BOOST_CHECK_EQUAL( twistPolar.vy() , twistGround.vy() );
        BOOST_CHECK_EQUAL( twistPolar.vh() , twistGround.vh() );
        BOOST_CHECK_EQUAL( twistPolar.date() , date );
        BOOST_CHECK_EQUAL( twistPolar.cov() , cov );
    }
}