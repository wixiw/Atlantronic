/*
 * uTest_Math.hpp
 *
 *  Created on: 30 mai 2011
 *      Author: ard
 */

#ifndef _UTEST_MATH_HPP_
#define _UTEST_MATH_HPP_

#include "math/math.hpp"
using namespace arp_math;

BOOST_AUTO_TEST_CASE( Math_normalizeAngle )
{
    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        double angle = (double) rand()/RAND_MAX * 10. - 5.;

        double res = normalizeAngle(angle);
        BOOST_CHECK_CLOSE( cos(res), cos(angle), 0.001f );
        BOOST_CHECK_CLOSE( sin(res), sin(angle), 0.001f );
        BOOST_CHECK( res >= -PI );
        BOOST_CHECK( res <= PI );
    }
}

BOOST_AUTO_TEST_CASE( Math_betweenZeroAndTwoPi )
{
    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        double angle = (double) rand()/RAND_MAX * 10. - 5.;

        double res = betweenZeroAndTwoPi(angle);
        BOOST_CHECK_CLOSE( cos(res), cos(angle), 0.001f );
        BOOST_CHECK_CLOSE( sin(res), sin(angle), 0.001f );
        BOOST_CHECK( res >= 0. );
        BOOST_CHECK( res <= 2. * PI );
    }
}

BOOST_AUTO_TEST_CASE( Math_betweenMinusPiAndPlusPi )
{
    BOOST_CHECK_EQUAL( betweenMinusPiAndPlusPi(0.), 0. );
    BOOST_CHECK_EQUAL( betweenMinusPiAndPlusPi(-PI), PI );
    BOOST_CHECK_EQUAL( betweenMinusPiAndPlusPi( PI), PI );

    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        double angle = (double) rand()/RAND_MAX * 10. - 5.;

        double res = betweenMinusPiAndPlusPi(angle);
        BOOST_CHECK_CLOSE( cos(res), cos(angle), 0.001f);
        BOOST_CHECK_CLOSE( sin(res), sin(angle), 0.001f);
        BOOST_CHECK( res >= -PI );
        BOOST_CHECK( res <= PI );
    }
}

BOOST_AUTO_TEST_CASE( Math_firstderivate )
{
    //periode nulle
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,0,0,0,0), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,0,0,1000,1000), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(123,52,0,1000,1000), 52 );

    //acc/decc nulles
    BOOST_CHECK_EQUAL( firstDerivateLimitation(1000,0,10,0,0), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,1000,10,0,0), 1000 );

    //Cas non limités
    BOOST_CHECK_EQUAL( firstDerivateLimitation(123,0,10,-1000,1000), 123 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-123,0,10,-1000,1000), -123 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(123,-50,10,-1000,1000), 123 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-123,50,10,-1000,1000), -123 );

    //Cas limités
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1,13,27), 27 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1,-13,27), 27 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1,-13,-27), 0 );//cas erreur
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,1, 13,-27), 0 );//cas erreur
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,13,27), -13 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,-13,27), -13 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,-13,-27), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,1,13,-27), 0 );

    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1,13,27), 87 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1,-13,27), 87 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1,-13,-27), 100 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,100,1, 13,-27), 100 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,13,27), -73 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,-13,27), -73 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,-13,-27), -100 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(0,-100,1,13,-27), -100 );


    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,-1,13,27), 0 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(100,0,2,-13,27), 54 );
    BOOST_CHECK_EQUAL( firstDerivateLimitation(-100,0,2,-13,27), -26 );

}

#endif /* UTEST_MATH_HPP_ */
