/*
 * uTest_CircleIdentif.hpp
 *
 *  Created on: 22 January 2012
 *      Author: boris
 */

#include "LSL/filters/CircleIdentif.hpp"

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_CircleIdentif )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::CircleIdentif::Params p;

    BOOST_CHECK( p.checkConsistency() );

    p.radius = 0.06;
    p.coeffs = std::vector<double>();
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radius = 0.06;
    p.coeffs = std::vector<double>();
    p.coeffs.push_back(2.0);
    p.coeffs.push_back(-0.1);
    BOOST_CHECK( p.checkConsistency() );

    p.radius = -0.06;
    p.coeffs = std::vector<double>();
    BOOST_CHECK( !(p.checkConsistency()) );

}

BOOST_AUTO_TEST_CASE( Test_DefaultParams_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,   1.01, 1.02, 1.03, 1.04 ;
    d.row(1) <<  1.3,   1.3,  1.3,  1.3,  1.3  ;
    d.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;

    rawScan.setPolarData(d);

    lsl::DetectedObject rawObject(rawScan);
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject);

    BOOST_CHECK_CLOSE(recoCircle.x(), 1.3 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.y(), 0.0, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.r(), 0.04, 1.f);

    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanRange(), 1.3, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTheta(), 0.0, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTime(), 1.02, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterRange(), 1.3 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTheta(), 0.0, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTime(), 1.02, 1.f);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK_CLOSE(m(0), 1.2998700036832864, 1.f);
    BOOST_CHECK_CLOSE(m(1), 0.0, 1.f);
    BOOST_CHECK_CLOSE(s(0), 0.00010876178953035609, 1.f);
    BOOST_CHECK_CLOSE(s(1), 0.01838373453045292, 1.f);

    MatrixXd pdata = recoCircle.getScan().getPolarData();
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_CLOSE(pdata(i,j), d(i,j), 1.f);
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_DefaultParams_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,   1.01, 1.02, 1.03, 1.04 ;
    d.row(1) <<  1.3,   1.3,  1.3,  1.3,  1.3  ;
    d.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;
    d.row(2) = d.row(2).array() + PI/2.;

    rawScan.setPolarData(d);

    lsl::DetectedObject rawObject(rawScan);
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject);

    BOOST_CHECK( abs(recoCircle.x()) < 0.000001 );
    BOOST_CHECK_CLOSE(recoCircle.y(), 1.3 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.r(), 0.04, 1.f);

    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanRange(), 1.3, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTheta(), PI/2, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTime(), 1.02, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterRange(), 1.3 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTheta(), PI/2, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTime(), 1.02, 1.f);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK( abs(m(0)) < 0.000001 );
    BOOST_CHECK_CLOSE(m(1), 1.2998700036832864, 1.f);
    BOOST_CHECK_CLOSE(s(0), 0.01838373453045292, 1.f);
    BOOST_CHECK_CLOSE(s(1), 0.00010876178953035609, 1.f);

    MatrixXd pdata = recoCircle.getScan().getPolarData();
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata(i,j), d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_OtherParams_1 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,   1.01, 1.02, 1.03, 1.04 ;
    d.row(1) <<  1.3,   1.3,  1.3,  1.3,  1.3  ;
    d.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;

    rawScan.setPolarData(d);

    lsl::DetectedObject rawObject(rawScan);

    lsl::CircleIdentif::Params p;
    p.radius = 0.06;
    p.coeffs = std::vector<double>();
    p.coeffs.push_back(1.0);
    p.coeffs.push_back(0.04);
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject, p);

    BOOST_CHECK_CLOSE(recoCircle.x(), 1.3 + 0.04, 1.f);
    BOOST_CHECK( abs(recoCircle.y()) < 0.000001 );
    BOOST_CHECK_CLOSE(recoCircle.r(), 0.06, 1.f);

    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanRange(), 1.3, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTheta(), 0.0, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTime(), 1.02, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterRange(), 1.3 + 0.04, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTheta(), 0.0, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTime(), 1.02, 1.f);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK_CLOSE(m(0), 1.2998700036832864, 1.f);
    BOOST_CHECK( abs(m(1)) < 0.000001 );
    BOOST_CHECK_CLOSE(s(0), 0.00010876178953035609, 1.f);
    BOOST_CHECK_CLOSE(s(1), 0.01838373453045292, 1.f);

    MatrixXd pdata = recoCircle.getScan().getPolarData();
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata(i,j), d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_OtherParams_2 )
{
    lsl::LaserScan rawScan;
    MatrixXd d(3, 5);
    d.row(0) <<  1.0,   1.01, 1.02, 1.03, 1.04 ;
    d.row(1) <<  1.3,   1.3,  1.3,  1.3,  1.3  ;
    d.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;
    d.row(2) = d.row(2).array() + PI/2.;

    rawScan.setPolarData(d);

    lsl::DetectedObject rawObject(rawScan);

    lsl::CircleIdentif::Params p;
    p.radius = 0.06;
    p.coeffs = std::vector<double>();
    p.coeffs.push_back(1.0);
    p.coeffs.push_back(0.04);
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject, p);

    BOOST_CHECK( abs(recoCircle.x()) < 0.000001 );
    BOOST_CHECK_CLOSE(recoCircle.y(), 1.3 + 0.04, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.r(), 0.06, 1.f);

    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanRange(), 1.3, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTheta(), PI/2, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCartesianMeanTime(), 1.02, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterRange(), 1.3 + 0.04, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTheta(), PI/2, 1.f);
    BOOST_CHECK_CLOSE(recoCircle.getApparentCenterTime(), 1.02, 1.f);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK( abs(m(0)) < 0.000001 );
    BOOST_CHECK_CLOSE(m(1), 1.2998700036832864, 1.f);
    BOOST_CHECK_CLOSE(s(0), 0.01838373453045292, 1.f);
    BOOST_CHECK_CLOSE(s(1), 0.00010876178953035609, 1.f);

    MatrixXd pdata = recoCircle.getScan().getPolarData();
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata(i,j), d(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE( Test_Vector )
{
    lsl::LaserScan rawScan1;
    MatrixXd d1(3, 5);
    d1.row(0) <<  1.0,   1.01, 1.02, 1.03, 1.04 ;
    d1.row(1) <<  1.3,   1.3,  1.3,  1.3,  1.3  ;
    d1.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;
    rawScan1.setPolarData(d1);
    lsl::DetectedObject rawObject1(rawScan1);

    lsl::LaserScan rawScan2;
    MatrixXd d2(3, 5);
    d2.row(0) <<  2.0,   2.01, 2.02, 2.03, 2.04 ;
    d2.row(1) <<  2.6,   2.6,  2.6,  2.6,  2.6  ;
    d2.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;
    d2.row(2) = d2.row(2).array() + PI/2.;
    rawScan2.setPolarData(d2);
    lsl::DetectedObject rawObject2(rawScan2);

    std::vector<lsl::DetectedObject> rawObjects;
    rawObjects.push_back(rawObject1);
    rawObjects.push_back(rawObject2);

    std::vector<lsl::DetectedCircle> recoCircles = lsl::CircleIdentif::apply(rawObjects);

    BOOST_CHECK_EQUAL( recoCircles.size(), 2);

    BOOST_CHECK_CLOSE(recoCircles[0].x(), 1.3 + 0.034, 1.f);
    BOOST_CHECK( abs(recoCircles[0].y()) < 0.000001 );
    BOOST_CHECK_CLOSE(recoCircles[0].r(), 0.04, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[0].getApparentCartesianMeanRange(), 1.3, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[0].getApparentCartesianMeanTheta(), 0.0, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[0].getApparentCartesianMeanTime(), 1.02, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[0].getApparentCenterRange(), 1.3 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[0].getApparentCenterTheta(), 0., 1.f);
    BOOST_CHECK_CLOSE(recoCircles[0].getApparentCenterTime(), 1.02, 1.f);
    Vector2 m1 = recoCircles[0].getCartesianMean();
    Vector2 s1 = recoCircles[0].getCartesianStddev();
    BOOST_CHECK_CLOSE(m1(0), 1.2998700036832864, 1.f);
    BOOST_CHECK( abs(m1(1)) < 0.000001 );
    BOOST_CHECK_CLOSE(s1(0), 0.00010876178953035609, 1.f);
    BOOST_CHECK_CLOSE(s1(1), 0.01838373453045292, 1.f);
    MatrixXd pdata1 = recoCircles[0].getScan().getPolarData();
    for (int i=0; i<pdata1.rows(); ++i)
    {
        for (int j=0; j<pdata1.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata1(i,j), d1(i,j));
        }
    }

    BOOST_CHECK( abs(recoCircles[1].x()) < 0.000001 );
    BOOST_CHECK_CLOSE(recoCircles[1].y(), 2.6 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].r(), 0.04, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].getApparentCartesianMeanRange(), 2.6, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].getApparentCartesianMeanTheta(), PI/2, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].getApparentCartesianMeanTime(), 2.02, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].getApparentCenterRange(), 2.6 + 0.034, 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].getApparentCenterTheta(), PI/2., 1.f);
    BOOST_CHECK_CLOSE(recoCircles[1].getApparentCenterTime(), 2.02, 1.f);
    Vector2 m2 = recoCircles[1].getCartesianMean();
    Vector2 s2 = recoCircles[1].getCartesianStddev();
    BOOST_CHECK( abs(m2(0)) < 0.000001 );
    BOOST_CHECK_CLOSE(m2(1), 2.5997400073665728, 1.f);
    BOOST_CHECK_CLOSE(s2(0), 0.036767469060905875, 1.f);
    BOOST_CHECK_CLOSE(s2(1), 0.00021752357906071218, 1.f);
    MatrixXd pdata2 = recoCircles[1].getScan().getPolarData();
    for (int i=0; i<pdata2.rows(); ++i)
    {
        for (int j=0; j<pdata2.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata2(i,j), d2(i,j));
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
