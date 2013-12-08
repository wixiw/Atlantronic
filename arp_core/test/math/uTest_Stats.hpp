/*
 * unittest_Stats.hpp
 *
 *  Created on: 23 February 2012
 *      Author: boris
 */

#ifndef _UTEST_STATS_HPP_
#define _UTEST_STATS_HPP_

#include "math/Stats.hpp"
using namespace arp_math;
using namespace Eigen;


BOOST_AUTO_TEST_SUITE( unittest_Stats )

BOOST_AUTO_TEST_CASE( mean_empty )
{
    VectorXd v(0);
    double m = mean(v);
    BOOST_CHECK_EQUAL(m, 0.);
}

BOOST_AUTO_TEST_CASE( mean_N1 )
{
    VectorXd v = VectorXd::Random(1);
    double m = mean(v);
    BOOST_CHECK_EQUAL(m, v[0]);
}

BOOST_AUTO_TEST_CASE( mean_N2 )
{
    VectorXd v = VectorXd::Random(2);
    double m = mean(v);
    BOOST_CHECK_EQUAL(m, (v[0] + v[1])/2.);
}

BOOST_AUTO_TEST_CASE( mean_N5 )
{
    VectorXd v = VectorXd::Random(5);
    double m = mean(v);
    BOOST_CHECK_EQUAL(m, v.sum()/5.);
}

BOOST_AUTO_TEST_CASE( stddev_empty )
{
    VectorXd v(0);
    double s = stddev(v);
    BOOST_CHECK_EQUAL(s, 0.);
}

BOOST_AUTO_TEST_CASE( stddev_N1 )
{
    VectorXd v = VectorXd::Random(1);
    double s = stddev(v);
    BOOST_CHECK_EQUAL(s, 0.);
}

BOOST_AUTO_TEST_CASE( stddev_N5 )
{
    VectorXd v(5);
    v << 3. , -2. ,  1.5,  7. ,  3.;
    double s = stddev(v);
    BOOST_CHECK_EQUAL(s, 2.8982753492378879);
}

BOOST_AUTO_TEST_CASE( median_empty )
{
    VectorXd v(0);
    double m = median(v);
    BOOST_CHECK_EQUAL(m, 0.);
}

BOOST_AUTO_TEST_CASE( median_N1 )
{
    VectorXd v = VectorXd::Random(1);
    double m = median(v);
    BOOST_CHECK_EQUAL(m, v[0]);
}

BOOST_AUTO_TEST_CASE( median_N2 )
{
    Eigen::VectorXd v(2);
    v(0) = 5.;
    v(1) = -1.;
    double result = median(v);
    BOOST_CHECK_EQUAL( result, -1. );
}

BOOST_AUTO_TEST_CASE( median_N3_1 )
{
    Eigen::VectorXd v(3);
    v(0) = 5.;
    v(1) = -1.;
    v(2) = 3.;
    double result = median(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_CASE( median_N3_2 )
{
    Eigen::VectorXd v(3);
    v(0) = 3.;
    v(1) = -1.;
    v(2) = 5.;
    double result = median(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_CASE( median_N3_3 )
{
    Eigen::VectorXd v(3);
    v(0) = 3.;
    v(1) = -1.;
    v(2) = 5.;
    double result = median(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_CASE( median_N5 )
{
    Eigen::VectorXd v(5);
    v(0) = 3.;
    v(1) = -1.;
    v(2) = 8.;
    v(3) = -3.;
    v(4) = 5.;
    double result = median(v);
    BOOST_CHECK_EQUAL( result, 3. );
}

BOOST_AUTO_TEST_SUITE_END()

#endif /* _UTEST_STATS_HPP_ */
