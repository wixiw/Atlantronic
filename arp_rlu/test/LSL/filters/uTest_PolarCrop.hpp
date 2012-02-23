/*
 * uTest_PolarCrop.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/PolarCrop.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_PolarCrop )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::PolarCrop::Params p;

    BOOST_CHECK( p.checkConsistency() );

    p.minRange = VectorXd::Zero(1);
    p.maxRange = 10.0 * VectorXd::Ones(1);
    p.minTheta = -PI;
    p.maxTheta = PI;
    BOOST_CHECK( p.checkConsistency() );

    p.minRange = -1.0 * VectorXd::Ones(1);
    p.maxRange = 10.0 * VectorXd::Ones(1);
    p.minTheta = -PI;
    p.maxTheta = PI;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.minRange = 10.0 * VectorXd::Ones(1);
    p.maxRange = VectorXd::Zero(1);
    p.minTheta = -PI;
    p.maxTheta = PI;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.minRange = VectorXd::Zero(10);
    p.maxRange = 10.0 * VectorXd::Ones(10);
    p.minTheta = -PI;
    p.maxTheta = PI;
    BOOST_CHECK( p.checkConsistency() );

    p.minRange = 10.0 * VectorXd::Ones(10);
    p.maxRange = VectorXd::Zero(10);
    p.minTheta = -PI;
    p.maxTheta = PI;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.minRange = VectorXd::Zero(1);
    p.maxRange = 10.0 * VectorXd::Ones(1);
    p.minTheta = PI;
    p.maxTheta = -PI;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( Test_DefaultParams )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan);

    MatrixXd filt = filtScan.getPolarData();

    VectorXi ifilt = VectorXi(3);
    ifilt << 0, 2, 4;
    BOOST_CHECK_EQUAL( filt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_CASE( Test_minRangeNul )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    lsl::PolarCrop::Params p;
    p.minRange = VectorXd::Zero(0);
    p.maxRange = 10.0 * VectorXd::Ones(1);
    p.minTheta = -PI;
    p.maxTheta = PI;
    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    for(int n = 0 ; n < filt.cols() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,n) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,n) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,n) );
    }
}

BOOST_AUTO_TEST_CASE( Test_maxRangeNul )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    lsl::PolarCrop::Params p;
    p.minRange = 0.1 * VectorXd::Ones(1);
    p.maxRange = VectorXd::Zero(0);
    p.minTheta = -PI;
    p.maxTheta = PI;
    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    for(int n = 0 ; n < filt.cols() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,n) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,n) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,n) );
    }
}

BOOST_AUTO_TEST_CASE( Test_Range_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    lsl::PolarCrop::Params p;
    p.minRange = VectorXd::Zero(1);
    p.maxRange = 10.0 * VectorXd::Ones(1);
    p.minTheta = -PI;
    p.maxTheta = PI;
    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    VectorXi ifilt = VectorXi(4);
    ifilt << 0, 1, 2, 4;
    BOOST_CHECK_EQUAL( filt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_CASE( Test_Range_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    lsl::PolarCrop::Params p;
    p.minRange = VectorXd::Zero(1);
    p.maxRange = 3.0 * VectorXd::Ones(1);
    p.minTheta = -PI;
    p.maxTheta = PI;
    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    VectorXi ifilt = VectorXi(3);
    ifilt << 0, 1, 4;
    BOOST_CHECK_EQUAL( filt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_CASE( Test_Range_3 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    VectorXd minr(5);
    minr << 0.5, 0.3, 1.1, 1.0, 1.5;
    VectorXd maxr(5);
    maxr << 2.0, 0.5, 3.1, 12.0, 2.1;

    lsl::PolarCrop::Params p;
    p.minRange = minr;
    p.maxRange = maxr;
    p.minTheta = -PI;
    p.maxTheta = PI;
    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    VectorXi ifilt = VectorXi(3);
    ifilt << 0, 3, 4;
    BOOST_CHECK_EQUAL( filt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_CASE( Test_Theta )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,  1.01, 1.02, 1.03, 1.04;
    d.row(1) <<  1.3,  0.05, 3.2, 11.5,  2.01;
    d.row(2) << -2.65, -1.3, 0.1,  0.7,  2.7;

    rawScan.setPolarData(d);

    lsl::PolarCrop::Params p;
    p.minRange = VectorXd::Zero(1);
    p.maxRange = 12.0 * VectorXd::Ones(1);
    p.minTheta = -2.0;
    p.maxTheta = 0.5;
    lsl::LaserScan filtScan = lsl::PolarCrop::apply(rawScan, p);

    MatrixXd filt = filtScan.getPolarData();

    VectorXi ifilt = VectorXi(2);
    ifilt << 1, 2;
    BOOST_CHECK_EQUAL( filt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( filt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( filt(2,n), d(2,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_SUITE_END()
