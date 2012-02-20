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
    p.rangeDelta = 0.04;
    BOOST_CHECK( p.checkConsistency() );

    p.radius = -0.06;
    p.rangeDelta = -0.08;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radius =  0.06;
    p.rangeDelta = 0.08;
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

    BOOST_CHECK_EQUAL(recoCircle.x(), 1.3 + 0.034);
    BOOST_CHECK_EQUAL(recoCircle.y(), 0.0);
    BOOST_CHECK_EQUAL(recoCircle.r(), 0.04);

    BOOST_CHECK_EQUAL(recoCircle.getApparentRange(), 1.3 + 0.034);
    BOOST_CHECK_EQUAL(recoCircle.getApparentTheta(), 0.0);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK_EQUAL(m(0), 1.3);
    BOOST_CHECK_EQUAL(m(1), 0.0);
    BOOST_CHECK_EQUAL(s(0), 0.0);
    BOOST_CHECK_EQUAL(s(1), 0.014142135623730951);

    MatrixXd pdata = recoCircle.getScan().getPolarData();
    for (int i=0; i<pdata.rows(); ++i)
    {
        for (int j=0; j<pdata.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata(i,j), d(i,j));
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

    BOOST_CHECK_EQUAL(recoCircle.x(), 0.0);
    BOOST_CHECK_EQUAL(recoCircle.y(), 1.3 + 0.034);
    BOOST_CHECK_EQUAL(recoCircle.r(), 0.04);

    BOOST_CHECK_EQUAL(recoCircle.getApparentRange(), 1.3 + 0.034);
    BOOST_CHECK_EQUAL(recoCircle.getApparentTheta(), PI/2);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK_EQUAL(m(0), 0.0);
    BOOST_CHECK_EQUAL(m(1), 1.3);
    BOOST_CHECK_EQUAL(s(0), 0.014142135623730951);
    BOOST_CHECK_EQUAL(s(1), 0.0);

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
    p.rangeDelta = 0.04;
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject);

    BOOST_CHECK_EQUAL(recoCircle.x(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircle.y(), 0.0);
    BOOST_CHECK_EQUAL(recoCircle.r(), 0.06);

    BOOST_CHECK_EQUAL(recoCircle.getApparentRange(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircle.getApparentTheta(), 0.0);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK_EQUAL(m(0), 1.3);
    BOOST_CHECK_EQUAL(m(1), 0.0);
    BOOST_CHECK_EQUAL(s(0), 0.0);
    BOOST_CHECK_EQUAL(s(1), 0.014142135623730951);

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
    p.rangeDelta = 0.04;
    lsl::DetectedCircle recoCircle = lsl::CircleIdentif::apply(rawObject);

    BOOST_CHECK_EQUAL(recoCircle.x(), 0.0);
    BOOST_CHECK_EQUAL(recoCircle.y(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircle.r(), 0.06);

    BOOST_CHECK_EQUAL(recoCircle.getApparentRange(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircle.getApparentTheta(), PI/2);

    Vector2 m = recoCircle.getCartesianMean();
    Vector2 s = recoCircle.getCartesianStddev();
    BOOST_CHECK_EQUAL(m(0), 0.0);
    BOOST_CHECK_EQUAL(m(1), 1.3);
    BOOST_CHECK_EQUAL(s(0), 0.014142135623730951);
    BOOST_CHECK_EQUAL(s(1), 0.0);

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
    d2.row(0) <<  1.0,   1.01, 1.02, 1.03, 1.04 ;
    d2.row(1) <<  1.3,   1.3,  1.3,  1.3,  1.3  ;
    d2.row(2) << -0.02, -0.01, 0.00, 0.01, 0.02 ;
    d2.row(2) = d2.row(2).array() + PI/2.;
    rawScan2.setPolarData(d2);
    lsl::DetectedObject rawObject2(rawScan1);

    std::vector<lsl::DetectedObject> rawObjects;
    rawObjects.push_back(rawObject1);
    rawObjects.push_back(rawObject2);

    std::vector<lsl::DetectedCircle> recoCircles = lsl::CircleIdentif::apply(rawObjects);

    BOOST_CHECK_EQUAL( recoCircles.size(), 2);

    BOOST_CHECK_EQUAL(recoCircles[0].x(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircles[0].y(), 0.0);
    BOOST_CHECK_EQUAL(recoCircles[0].r(), 0.06);
    BOOST_CHECK_EQUAL(recoCircles[0].getApparentRange(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircles[0].getApparentTheta(), 0.0);
    Vector2 m1 = recoCircles[0].getCartesianMean();
    Vector2 s1 = recoCircles[0].getCartesianStddev();
    BOOST_CHECK_EQUAL(m1(0), 1.3);
    BOOST_CHECK_EQUAL(m1(1), 0.0);
    BOOST_CHECK_EQUAL(s1(0), 0.0);
    BOOST_CHECK_EQUAL(s1(1), 0.014142135623730951);
    MatrixXd pdata1 = recoCircles[0].getScan().getPolarData();
    for (int i=0; i<pdata1.rows(); ++i)
    {
        for (int j=0; j<pdata1.cols(); ++j)
        {
            BOOST_CHECK_EQUAL(pdata1(i,j), d1(i,j));
        }
    }

    BOOST_CHECK_EQUAL(recoCircles[1].x(), 0.0);
    BOOST_CHECK_EQUAL(recoCircles[1].y(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircles[1].r(), 0.06);
    BOOST_CHECK_EQUAL(recoCircles[1].getApparentRange(), 1.3 + 0.04);
    BOOST_CHECK_EQUAL(recoCircles[1].getApparentTheta(), PI/2);
    Vector2 m2 = recoCircles[1].getCartesianMean();
    Vector2 s2 = recoCircles[1].getCartesianStddev();
    BOOST_CHECK_EQUAL(m2(0), 0.0);
    BOOST_CHECK_EQUAL(m2(1), 1.3);
    BOOST_CHECK_EQUAL(s2(0), 0.014142135623730951);
    BOOST_CHECK_EQUAL(s2(1), 0.0);
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
