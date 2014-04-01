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

    t = { 1,500000000 };
    BOOST_CHECK_EQUAL(timespec2Double(t),1.5);

    t = { 1,234567891 };
    BOOST_CHECK_CLOSE(timespec2Double(t),1.234567891,1E-10);

    t = { 1, 1750000000 };
    BOOST_CHECK_EQUAL(timespec2Double(t),2.75);
}

BOOST_AUTO_TEST_CASE( Math_double2Timespec )
{
    timespec t = double2Timespec(0.0);
    BOOST_CHECK_EQUAL(t.tv_sec,0);
    BOOST_CHECK_EQUAL(t.tv_nsec,0);

    t = double2Timespec(1.0);
    BOOST_CHECK_EQUAL(t.tv_sec,1);
    BOOST_CHECK_EQUAL(t.tv_nsec,0);

    t = double2Timespec(1.5);
    BOOST_CHECK_EQUAL(t.tv_sec,1);
    BOOST_CHECK_EQUAL(t.tv_nsec,5*1E8);

    t = double2Timespec(1.234567891);
    BOOST_CHECK_EQUAL(t.tv_sec,1);
    BOOST_CHECK_CLOSE((double)t.tv_nsec,double(234567891),1E-5);

    t = double2Timespec(1.2345678916666666666);
    BOOST_CHECK_EQUAL(t.tv_sec,1);
    BOOST_CHECK_CLOSE((double)t.tv_nsec,234567891.0,1E-5);

    BOOST_CHECK_CLOSE(timespec2Double(double2Timespec(4.8765)), 4.8765, 1E-5);
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

BOOST_AUTO_TEST_CASE( Math_incrementTime )
{
    timespec tbegin;
    tbegin.tv_sec = 4;
    tbegin.tv_nsec = 234567891;
    double increment = 0.4;
    incrementTime(tbegin, increment);
    BOOST_CHECK_EQUAL(tbegin.tv_sec,4);
    BOOST_CHECK_CLOSE((double)tbegin.tv_nsec,634567891,1E-5);

    tbegin.tv_sec = 4;
    tbegin.tv_nsec = 234567891;
    increment = 1.2;
    incrementTime(tbegin, increment);
    BOOST_CHECK_EQUAL(tbegin.tv_sec,5);
    BOOST_CHECK_CLOSE((double)tbegin.tv_nsec,434567891,1E-5);

    tbegin.tv_sec = 4;
    tbegin.tv_nsec = 234567891;
    increment = 1.9;
    incrementTime(tbegin, increment);
    BOOST_CHECK_EQUAL(tbegin.tv_sec,6);
    BOOST_CHECK_CLOSE((double)tbegin.tv_nsec,134567891,1E-5);
}


#endif /* UTEST_MATH_HPP_ */
