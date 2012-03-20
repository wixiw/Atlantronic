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

#endif /* UTEST_MATH_HPP_ */
