/*
 * uTest_DetectedCircle.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/objects/DetectedCircle.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_DetectedCircle )

BOOST_AUTO_TEST_CASE( Constructor_default )
{
    lsl::DetectedCircle obj;

    BOOST_CHECK_EQUAL( obj.getApparentCartesianMeanRange(), 0.);
    BOOST_CHECK_EQUAL( obj.getApparentCartesianMeanTheta(), 0.);
    BOOST_CHECK_EQUAL( obj.getApparentCartesianMeanTime(), 0.);

    BOOST_CHECK_EQUAL( obj.getApparentCenterRange(), 0.);
    BOOST_CHECK_EQUAL( obj.getApparentCenterTheta(), 0.);
    BOOST_CHECK_EQUAL( obj.getApparentCenterTime(), 0.);

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

    BOOST_CHECK_EQUAL( obj.x(), 0.);
    BOOST_CHECK_EQUAL( obj.y(), 0.);
    BOOST_CHECK_EQUAL( obj.r(), 1.);
}

BOOST_AUTO_TEST_CASE( Constructor_copy_DetectedObject )
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

    lsl::DetectedCircle dcircle(obj);

    dcircle.setApparentCenterRange(1.0);
    dcircle.setApparentCenterTheta(-2.0);
    dcircle.setApparentCenterTime(10.0);

    BOOST_CHECK_CLOSE( dcircle.getApparentCartesianMeanRange(), 0.67314560089181297, 1.f);
    BOOST_CHECK_CLOSE( dcircle.getApparentCartesianMeanTheta(), -2.761086276477428, 1.f);
    BOOST_CHECK_CLOSE( dcircle.getApparentCartesianMeanTime(), 0.2, 1.f);
    arp_math::Vector2 pov = dcircle.getApparentPointOfView();
    BOOST_CHECK_CLOSE( pov.x(), 1.0, 1.f);
    BOOST_CHECK_CLOSE( pov.y(),-1.0, 1.f);
    BOOST_CHECK_CLOSE( dcircle.getApparentAngleOfView(), PI, 1.f);

    BOOST_CHECK_CLOSE( dcircle.getApparentCenterRange(), 1., 1.f);
    BOOST_CHECK_CLOSE( dcircle.getApparentCenterTheta(), -2., 1.f);
    BOOST_CHECK_CLOSE( dcircle.getApparentCenterTime(), 10., 1.f);

    arp_math::Vector2 m = dcircle.getCartesianMean();
    arp_math::Vector2 s = dcircle.getCartesianStddev();
    BOOST_CHECK_CLOSE( m(0), 1.625 , 1.f);
    BOOST_CHECK_CLOSE( m(1), -0.75, 1.f);
    BOOST_CHECK_CLOSE( s(0), 1.38631706, 1.f);
    BOOST_CHECK_CLOSE( s(1), 1.78535711, 1.f);

    Eigen::MatrixXd pdata = dcircle.getScan().getPolarData();

    BOOST_CHECK_EQUAL( pdata.rows(), 3);
    BOOST_CHECK_EQUAL( pdata.cols(), 4);

    for (int i=0; i<pdata.rows(); ++i) {
        for (int j=0; j<pdata.cols(); ++j) {
            BOOST_CHECK_EQUAL(pdata(i,j),d(i,j));
        }
    }

    BOOST_CHECK_EQUAL( dcircle.x(), 0.);
    BOOST_CHECK_EQUAL( dcircle.y(), 0.);
    BOOST_CHECK_EQUAL( dcircle.r(), 1.);

}

BOOST_AUTO_TEST_SUITE_END()
