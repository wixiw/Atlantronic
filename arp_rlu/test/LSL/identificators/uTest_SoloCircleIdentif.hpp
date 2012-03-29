/*
 * uTest_SoloCircleIdentif.hpp
 *
 *  Created on: 23 Mars 2012
 *      Author: boris
 */

#include "LSL/identificators/SoloCircleIdentif.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_SoloCircleIdentif )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::SoloCircleIdentif::Params p;
    BOOST_CHECK( p.checkConsistency() );

    p.radiusTolerance = 0.1;
    p.distanceTolerance = 0.6;
    BOOST_CHECK( p.checkConsistency() );

    p.radiusTolerance = -0.1;
    p.distanceTolerance = 0.6;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = -0.6;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( Test_Case_Empty )
{

    std::vector<lsl::Circle> refCircles;
    std::vector<lsl::DetectedCircle> detectedCircles;
    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = lsl::SoloCircleIdentif::apply( detectedCircles, refCircles);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Trivial)
{
    std::vector<lsl::Circle> refCircles;
    refCircles.push_back( lsl::Circle(1., -2., 0.04));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y( -2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::SoloCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = lsl::SoloCircleIdentif::apply( detectedCircles, refCircles, p );

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].second.x(), refCircles[0].x() );
    BOOST_CHECK_EQUAL( res[0].second.y(), refCircles[0].y() );
    BOOST_CHECK_EQUAL( res[0].second.r(), refCircles[0].r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Dual_1)
{
    lsl::SoloCircleIdentif obj;

    std::vector<lsl::Circle> refCircles;
    refCircles.push_back( lsl::Circle(1., -2., 0.04));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y( -2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  1.01 );
    dc2.y( -2.01 );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::SoloCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = obj.apply( detectedCircles, refCircles, p );

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].second.x(), refCircles[0].x() );
    BOOST_CHECK_EQUAL( res[0].second.y(), refCircles[0].y() );
    BOOST_CHECK_EQUAL( res[0].second.r(), refCircles[0].r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Dual_2)
{
    lsl::SoloCircleIdentif obj;

    std::vector<lsl::Circle> refCircles;
    refCircles.push_back( lsl::Circle(1., -2., 0.04));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.01 );
    dc1.y( -2.01  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  1. );
    dc2.y( -2. );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::SoloCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = obj.apply( detectedCircles, refCircles, p );

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].second.x(), refCircles[0].x() );
    BOOST_CHECK_EQUAL( res[0].second.y(), refCircles[0].y() );
    BOOST_CHECK_EQUAL( res[0].second.r(), refCircles[0].r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Radius_Pb)
{
    std::vector<lsl::Circle> refCircles;
    refCircles.push_back( lsl::Circle(1., -2., 0.04));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y( -2.  );
    dc1.r(  0.1 );
    detectedCircles.push_back( dc1 );

    lsl::SoloCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = lsl::SoloCircleIdentif::apply( detectedCircles, refCircles, p );

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Distance_Pb)
{
    std::vector<lsl::Circle> refCircles;
    refCircles.push_back( lsl::Circle(1., -2., 0.04));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.2 );
    dc1.y( -2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::SoloCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = lsl::SoloCircleIdentif::apply( detectedCircles, refCircles, p );

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Complex)
{
    std::vector<lsl::Circle> refCircles;
    refCircles.push_back( lsl::Circle(1., -2., 0.04));
    refCircles.push_back( lsl::Circle(10., 3., 0.06));
    refCircles.push_back( lsl::Circle(-4., 7., 0.06));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.01 );
    dc1.y( -2.0  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  10.0  );
    dc2.y(   3.0  );
    dc2.r(   0.4  );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  11.0  );
    dc3.y(   3.01 );
    dc3.r(   0.05 );
    detectedCircles.push_back( dc3 );

    lsl::DetectedCircle dc4;
    dc4.x(  10.0  );
    dc4.y(   4.01 );
    dc4.r(   0.05 );
    detectedCircles.push_back( dc4 );

    lsl::DetectedCircle dc5;
    dc5.x(  1.09 );
    dc5.y( -2.0  );
    dc5.r(  0.04 );
    detectedCircles.push_back( dc5 );

    lsl::SoloCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res = lsl::SoloCircleIdentif::apply( detectedCircles, refCircles, p );

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].second.x(), refCircles[0].x() );
    BOOST_CHECK_EQUAL( res[0].second.y(), refCircles[0].y() );
    BOOST_CHECK_EQUAL( res[0].second.r(), refCircles[0].r() );

}

BOOST_AUTO_TEST_SUITE_END()
