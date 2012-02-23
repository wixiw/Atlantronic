/*
 * uTest_CartesianCrop.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/CartesianCrop.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_CartesianCrop )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::CartesianCrop::Params p;
    BOOST_CHECK( p.checkConsistency() );

    p.minX = 1.0;
    p.maxX = 2.0;
    p.minY = -1.0;
    p.maxY = -0.5;
    BOOST_CHECK( p.checkConsistency() );

    p.minX = -1.0;
    p.maxX = 2.0;
    p.minY = 1.0;
    p.maxY = 3.5;
    BOOST_CHECK( p.checkConsistency() );

    p.minX = -1.0;
    p.maxX = 2.0;
    p.minY = 2.0;
    p.maxY = 1.5;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.minX = 1.0;
    p.maxX = -2.0;
    p.minY = 0.0;
    p.maxY = 1.5;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( Test_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<   1.0,    1.1 ,  1.2,   1.3,   1.4  ;
    d.row(1) <<   0.0,    1.0 ,  2.5,   2.0,   0.5  ;
    d.row(2) << -PI/2., -PI/4.,  0.0,  PI/4.,  PI/2.;

    rawScan.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(5);
    rawScan.computeCartesianData(tt, xx, yy, hh);

    MatrixXd cart = rawScan.getCartesianData();

    lsl::LaserScan filtScan = lsl::CartesianCrop::apply(rawScan);

    MatrixXd polarFilt = filtScan.getPolarData();
    MatrixXd cartFilt = filtScan.getCartesianData();

    VectorXi ifilt = VectorXi(3);
    ifilt << 0, 1, 4;
    BOOST_CHECK_EQUAL( polarFilt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( polarFilt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( polarFilt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( polarFilt(2,n), d(2,ifilt(n)) );

        BOOST_CHECK_EQUAL( cartFilt(1,n), cart(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(2,n), cart(2,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(3,n), cart(3,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(4,n), cart(4,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(5,n), cart(5,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_CASE( Test_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<   1.0,    1.1 ,  1.2,   1.3,   1.4  ;
    d.row(1) <<   0.0,    1.0 ,  2.5,   2.0,   0.5  ;
    d.row(2) << -PI/2., -PI/4.,  0.0,  PI/4.,  PI/2.;

    rawScan.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(5);
    rawScan.computeCartesianData(tt, xx, yy, hh);

    MatrixXd cart = rawScan.getCartesianData();

    lsl::CartesianCrop::Params p;
    p.minX = -1.0;
    p.maxX = 1.0;
    p.minY = -1.0;
    p.maxY = 0.1;
    lsl::LaserScan filtScan = lsl::CartesianCrop::apply(rawScan, p);

    MatrixXd polarFilt = filtScan.getPolarData();
    MatrixXd cartFilt = filtScan.getCartesianData();

    VectorXi ifilt = VectorXi(2);
    ifilt << 0, 1;
    BOOST_CHECK_EQUAL( polarFilt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( polarFilt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( polarFilt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( polarFilt(2,n), d(2,ifilt(n)) );

        BOOST_CHECK_EQUAL( cartFilt(1,n), cart(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(2,n), cart(2,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(3,n), cart(3,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(4,n), cart(4,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(5,n), cart(5,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_CASE( Test_3 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<   1.0,    1.1 ,  1.2,   1.3,   1.4  ;
    d.row(1) <<   0.0,    1.0 ,  2.5,   2.0,   0.5  ;
    d.row(2) << -PI/2., -PI/4.,  0.0,  PI/4.,  PI/2.;

    rawScan.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(5);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(5);
    rawScan.computeCartesianData(tt, xx, yy, hh);

    MatrixXd cart = rawScan.getCartesianData();

    lsl::CartesianCrop::Params p;
    p.minX = 0.5;
    p.maxX = 3.5;
    p.minY = -1.0;
    p.maxY = 1.0;
    lsl::LaserScan filtScan = lsl::CartesianCrop::apply(rawScan, p);

    MatrixXd polarFilt = filtScan.getPolarData();
    MatrixXd cartFilt = filtScan.getCartesianData();

    VectorXi ifilt = VectorXi(2);
    ifilt << 1, 2;
    BOOST_CHECK_EQUAL( polarFilt.cols(), ifilt.size() );
    for(int n = 0 ; n < ifilt.size() ; n++)
    {
        BOOST_CHECK_EQUAL( polarFilt(0,n), d(0,ifilt(n)) );
        BOOST_CHECK_EQUAL( polarFilt(1,n), d(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( polarFilt(2,n), d(2,ifilt(n)) );

        BOOST_CHECK_EQUAL( cartFilt(1,n), cart(1,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(2,n), cart(2,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(3,n), cart(3,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(4,n), cart(4,ifilt(n)) );
        BOOST_CHECK_EQUAL( cartFilt(5,n), cart(5,ifilt(n)) );
    }
}

BOOST_AUTO_TEST_SUITE_END()
