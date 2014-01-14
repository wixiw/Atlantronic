/*
 * uTest_Math.hpp
 *
 *  Created on: 30 mai 2011
 *      Author: ard
 */

#ifndef _UTEST_MATH_TIME_HPP_
#define _UTEST_MATH_TIME_HPP_

#include "math/math.hpp"
using namespace arp_math;

BOOST_AUTO_TEST_CASE( Math_timespec2Double )
{
    timespec t = { 0,0 };
    BOOST_CHECK_EQUAL(timespec2Double(t),0.0);

    t = { 1,0 };
    BOOST_CHECK_EQUAL(timespec2Double(t),1.0);

    t = { 1,5*1E8 };
    BOOST_CHECK_EQUAL(timespec2Double(t),1.5);

    t = { 1,234567891 };
    BOOST_CHECK_CLOSE(timespec2Double(t),1.234567891,1E-10);

    t = { 1, 1.75*1E9 };
    BOOST_CHECK_EQUAL(timespec2Double(t),2.75);
}


BOOST_AUTO_TEST_CASE( Math_timespecDelta )
{
    timespec tbegin = { 0,0 };
    timespec tend = { 0,0 };
    BOOST_CHECK_EQUAL(delta_t(tbegin,tend), 0.0);

    tbegin = { 0,0 };
    tend = { 4,0 };
    BOOST_CHECK_EQUAL(delta_t(tbegin,tend), 4.0);

    tbegin = { 0,0 };
    tend = { 4,234567891 };
    BOOST_CHECK_CLOSE(delta_t(tbegin,tend), 4.234567891,1E-10);

    tbegin = { 0,0 };
    tend = { 1, 1.75*1E9 };
    BOOST_CHECK_CLOSE(delta_t(tbegin,tend), 2.75,1E-10);

    tbegin = { 0,0 };
    tend = { -4,0 };
    BOOST_CHECK_EQUAL(delta_t(tbegin,tend), -4.0);

    tbegin = { 0,0 };
    tend = { -4,234567891 };
    BOOST_CHECK_CLOSE(delta_t(tbegin,tend), -3.765432109,1E-10);

    tbegin = { 0,0 };
    tend = { -1, 1.75*1E9 };
    BOOST_CHECK_CLOSE(delta_t(tbegin,tend), 0.75,1E-10);


    tbegin = { 4,0 };
    tend = { 0,0 };
    BOOST_CHECK_EQUAL(delta_t(tbegin,tend), -4.0);

    tbegin = { 4,234567891 };
    tend = { 0,0 };
    BOOST_CHECK_CLOSE(delta_t(tbegin,tend), -4.234567891,1E-10);

    tbegin = { 1, 1.75*1E9 };
    tend = { 0,0 };
    BOOST_CHECK_CLOSE(delta_t(tbegin,tend), -2.75,1E-10);

    tbegin = { 0,5*1E8 };
    tend = { 1,0 };
    BOOST_CHECK_EQUAL(delta_t(tbegin,tend), 0.5);

}

#endif /* UTEST_MATH_HPP_ */
