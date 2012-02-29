/*
 * uTest_DetectedObject.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/objects/DetectedObject.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_DetectedObject )

BOOST_AUTO_TEST_CASE( Constructor_default )
{
    lsl::DetectedObject obj;

    BOOST_CHECK_EQUAL( obj.getApparentCartesianMeanRange(), 0.);
    BOOST_CHECK_EQUAL( obj.getApparentCartesianMeanTheta(), 0.);
    BOOST_CHECK_EQUAL( obj.getApparentCartesianMeanTime(), 0.);

    arp_math::Vector2 m = obj.getCartesianMean();
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_EQUAL( m(0), 0.);
    BOOST_CHECK_EQUAL( m(1), 0.);
    BOOST_CHECK_EQUAL( s(0), 0.);
    BOOST_CHECK_EQUAL( s(1), 0.);

    lsl::LaserScan ls = obj.getScan();
    Eigen::MatrixXd pdata = ls.getPolarData();
    Eigen::MatrixXd cdata = ls.getCartesianData();

    BOOST_CHECK_EQUAL( ls.getSize(), 0);
    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 0);
    BOOST_CHECK_EQUAL( cdata.rows(), 6);
    BOOST_CHECK_EQUAL( cdata.cols(), 0);

}

BOOST_AUTO_TEST_CASE( Constructor_copy )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  ,  0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj1;
    obj1.setScan(ls);

    lsl::DetectedObject obj2(obj1);

    BOOST_CHECK_CLOSE( obj2.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( obj2.getApparentCartesianMeanTheta(), -1.9513027039072615, 1.f);
    BOOST_CHECK_CLOSE( obj2.getApparentCartesianMeanTime(), 0.2, 1.f);

    arp_math::Vector2 m = obj2.getCartesianMean();
    arp_math::Vector2 s = obj2.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), -0.25, 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.625, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.08972474, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.38631706, 1.f);

    Eigen::MatrixXd pdata = obj2.getScan().getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 4);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Constructor_scan )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTheta(), -1.9513027039072615, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTime(), 0.2, 1.f);

    arp_math::Vector2 m = obj.getCartesianMean();
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), -0.25, 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.625, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.08972474, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.38631706, 1.f);

    Eigen::MatrixXd pdata = obj.getScan().getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 4);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( setScan )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTheta(), -1.9513027039072615, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTime(), 0.2, 1.f);

    arp_math::Vector2 m = obj.getCartesianMean();
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), -0.25, 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.625, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.08972474, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.38631706, 1.f);

    Eigen::MatrixXd pdata = obj.getScan().getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 4);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( getScan )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    Eigen::MatrixXd pdata = obj.getScan().getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 4);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( getCartesianMean )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    arp_math::Vector2 m = obj.getCartesianMean();
    BOOST_CHECK_CLOSE( m(0), -0.25, 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.625, 1.f);
}

BOOST_AUTO_TEST_CASE( getCartesianStddev )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_CLOSE( s(0), 1.08972474, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.38631706, 1.f);
}

BOOST_AUTO_TEST_CASE( getApparentCartesianMeanRange )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
}

BOOST_AUTO_TEST_CASE( getApparentCartesianMeanTheta )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTheta(), -1.9513027039072615, 1.f);
}

BOOST_AUTO_TEST_CASE( getApparentCartesianMeanTime )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTime(), 0.2, 1.f);
}

BOOST_AUTO_TEST_CASE( getApparentPointOfView )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    arp_math::Vector2 pov = obj.getApparentPointOfView();
    BOOST_CHECK_EQUAL( pov.x(), 0.0);
    BOOST_CHECK_EQUAL( pov.y(), 0.0);
}

BOOST_AUTO_TEST_CASE( getApparentAngleOfView )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_EQUAL( obj.getApparentAngleOfView(), 0.0);
}

BOOST_AUTO_TEST_CASE( computeStatistics_1 )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(4);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTheta(), -1.9513027039072615, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTime(), 0.2, 1.f);

    arp_math::Vector2 m = obj.getCartesianMean();
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), -0.25 , 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.625, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.08972474, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.38631706, 1.f);
}

BOOST_AUTO_TEST_CASE( computeStatistics_2 )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) << 0.1 ,  0.2  , 0.3  ,    0.4  ;
    d.row(1) << 1.0,   0.5  ,  2.0 ,    3.0  ;
    d.row(2) << 0. , M_PI/2., M_PI , -M_PI/2.;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTheta(), -1.9513027039072615, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTime(), 0.2, 1.f);

    arp_math::Vector2 m = obj.getCartesianMean();
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), -0.25, 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.625, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.08972474, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.38631706, 1.f);
}

BOOST_AUTO_TEST_CASE( computeStatistics_3 )
{
    Eigen::MatrixXd d = MatrixXd::Random(3, 4);
    d.row(0) <<  0.1 ,    0.2  , 0.3  ,    0.4  ;
    d.row(1) <<  3.0 ,    0.5  ,  2.0 ,    3.0  ;
    d.row(2) << -M_PI/2., 0.0  , M_PI/2., M_PI  ;
    lsl::LaserScan ls;
    ls.setPolarData(d);

    Eigen::VectorXd tt = d.row(0);
    Eigen::VectorXd xx = Eigen::VectorXd::Constant(4, 1.0);
    Eigen::VectorXd yy = Eigen::VectorXd::Constant(4,-1.0);
    Eigen::VectorXd hh = Eigen::VectorXd::Constant(4,PI);
    ls.computeCartesianData(tt, xx, yy, hh);

    lsl::DetectedObject obj;
    obj.setScan(ls);

    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTheta(), -2.761086276477428, 1.f);
    BOOST_CHECK_CLOSE( obj.getApparentCartesianMeanTime(), 0.2, 1.f);
    arp_math::Vector2 pov = obj.getApparentPointOfView();
    BOOST_CHECK_EQUAL( pov.x(), 1.0);
    BOOST_CHECK_EQUAL( pov.y(),-1.0);
    BOOST_CHECK_EQUAL( obj.getApparentAngleOfView(), PI);

    arp_math::Vector2 m = obj.getCartesianMean();
    arp_math::Vector2 s = obj.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), 1.625 , 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.75, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.38631706, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.78535711, 1.f);
}

BOOST_AUTO_TEST_SUITE_END()
