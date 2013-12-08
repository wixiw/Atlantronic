/*
 * test_Pose2D.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

// TODO BMO: test unitaire pour l'opérateur <<

#include <filters/TwistDerivativeSat.hpp>
#include <iostream>
using namespace std;
using namespace arp_math;

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_trivial )
{
	Twist2D t = TwistDerivativeSat::apply(Twist2D(), Twist2D(), Vector3(0., 0., 0.), 1.0);
	BOOST_CHECK_EQUAL( t.vx(),  0. );
	BOOST_CHECK_EQUAL( t.vy(),  0. );
	BOOST_CHECK_EQUAL( t.vh(),  0. );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_DerivativeNull )
{
    Twist2D target(1., 2., 3.);
    Twist2D previous = target;
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(target, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  1. );
    BOOST_CHECK_EQUAL( t.vy(),  2. );
    BOOST_CHECK_EQUAL( t.vh(),  3. );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_1 )
{
    Twist2D target(1., 2., 3.);
    Twist2D previous(2., 2., 3.);
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(target, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  1.5 );
    BOOST_CHECK_EQUAL( t.vy(),  2. );
    BOOST_CHECK_EQUAL( t.vh(),  3. );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_2 )
{
    Twist2D current(1., 2., 3.);
    Twist2D previous(1., 3., 3.);
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(current, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  1. );
    BOOST_CHECK_EQUAL( t.vy(),  2.5 );
    BOOST_CHECK_EQUAL( t.vh(),  3. );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_3 )
{
    Twist2D target(1., 2., 3.);
    Twist2D previous(1., 2., 4.);
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(target, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  1. );
    BOOST_CHECK_EQUAL( t.vy(),  2. );
    BOOST_CHECK_EQUAL( t.vh(),  3.5 );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_4 )
{
    Twist2D target(1., 2., 3.);
    Twist2D previous(0., 2., 3.);
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(target, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  0.5 );
    BOOST_CHECK_EQUAL( t.vy(),  2. );
    BOOST_CHECK_EQUAL( t.vh(),  3. );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_5 )
{
    Twist2D target(1., 2., 3.);
    Twist2D previous(1., 1., 3.);
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(target, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  1. );
    BOOST_CHECK_EQUAL( t.vy(),  1.5 );
    BOOST_CHECK_EQUAL( t.vh(),  3. );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_6 )
{
    Twist2D target(1., 2., 3.);
    Twist2D previous(1., 2., 2.);
    Vector3 limits(1.,1.,1.);
    double period = 0.5;
    Twist2D t = TwistDerivativeSat::apply(target, previous, limits, period);
    BOOST_CHECK_EQUAL( t.vx(),  1. );
    BOOST_CHECK_EQUAL( t.vy(),  2. );
    BOOST_CHECK_EQUAL( t.vh(),  2.5 );
}

BOOST_AUTO_TEST_CASE( TwistDerivativeSat_7 )
{
    Twist2D target(53,66,12);
    Twist2D init(0,0,0);
    Vector3 limits(10,3,5.5);
    Twist2D ret;

    ret = TwistDerivativeSat::apply(target, init, limits, 1);
    BOOST_CHECK_CLOSE( ret.vx() ,  limits[0], 1E-6);
    BOOST_CHECK_CLOSE( ret.vy() ,  limits[1], 1E-6);
    BOOST_CHECK_CLOSE( ret.vh() ,  limits[2], 1E-6);

    target = Twist2D(1,2,3);
    ret = TwistDerivativeSat::apply(target, init, limits, 1);
    BOOST_CHECK_CLOSE( ret.vx() ,  1, 1E-6);
    BOOST_CHECK_CLOSE( ret.vy() ,  2, 1E-6);
    BOOST_CHECK_CLOSE( ret.vh() ,  3, 1E-6);

    target = Twist2D(53,66,12);
    init = Twist2D(1,2,3);
    ret = TwistDerivativeSat::apply(target, init, limits, 1);
    BOOST_CHECK_CLOSE( ret.vx() ,  limits[0]+1, 1E-6);
    BOOST_CHECK_CLOSE( ret.vy() ,  limits[1]+2, 1E-6);
    BOOST_CHECK_CLOSE( ret.vh() ,  limits[2]+3, 1E-6);

    target = Twist2D(2,4,6);
    init = Twist2D(1,2,3);
    ret = TwistDerivativeSat::apply(target, init, limits, 1);
    BOOST_CHECK_CLOSE( ret.vx() ,  2, 1E-6);
    BOOST_CHECK_CLOSE( ret.vy() ,  4, 1E-6);
    BOOST_CHECK_CLOSE( ret.vh() ,  6, 1E-6);
}
