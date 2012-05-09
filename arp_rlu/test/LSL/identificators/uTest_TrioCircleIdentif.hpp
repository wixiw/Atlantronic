/*
 * uTest_TrioCircleIdentif.hpp
 *
 *  Created on: 23 Mars 2012
 *      Author: boris
 */

#include "LSL/identificators/TrioCircleIdentif.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_TrioCircleIdentif )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::TrioCircleIdentif::Params p;
    BOOST_CHECK( p.checkConsistency() );

    p.radiusTolerance = 0.1;
    p.distanceTolerance = 0.6;
    p.maxLengthTolerance = 0.2;
    p.medLengthTolerance = 0.2;
    p.minLengthTolerance = 0.2;
    BOOST_CHECK( p.checkConsistency() );

    p.radiusTolerance = -0.1;
    p.distanceTolerance = 0.6;
    p.maxLengthTolerance = 0.2;
    p.medLengthTolerance = 0.2;
    p.minLengthTolerance = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = -0.6;
    p.maxLengthTolerance = 0.2;
    p.medLengthTolerance = 0.2;
    p.minLengthTolerance = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = 0.6;
    p.maxLengthTolerance = -0.2;
    p.medLengthTolerance = 0.2;
    p.minLengthTolerance = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = 0.6;
    p.maxLengthTolerance = 0.2;
    p.medLengthTolerance = -0.2;
    p.minLengthTolerance = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = 0.6;
    p.maxLengthTolerance = 0.2;
    p.medLengthTolerance = 0.2;
    p.minLengthTolerance = -0.2;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( Test_Case_Empty )
{
    std::vector< std::vector<lsl::Circle> > refTrio;
    std::vector< lsl::DetectedCircle> detectedCircles;
    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Trivial )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.5;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first[0].x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first[0].y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first[0].r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].first[1].x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first[1].y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first[1].r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].first[2].x(), dc3.x() );
    BOOST_CHECK_EQUAL( res[0].first[2].y(), dc3.y() );
    BOOST_CHECK_EQUAL( res[0].first[2].r(), dc3.r() );


    BOOST_CHECK_EQUAL( res[0].second[0].x(), refTrio[0][0].x() );
    BOOST_CHECK_EQUAL( res[0].second[0].y(), refTrio[0][0].y() );
    BOOST_CHECK_EQUAL( res[0].second[0].r(), refTrio[0][0].r() );

    BOOST_CHECK_EQUAL( res[0].second[1].x(), refTrio[0][1].x() );
    BOOST_CHECK_EQUAL( res[0].second[1].y(), refTrio[0][1].y() );
    BOOST_CHECK_EQUAL( res[0].second[1].r(), refTrio[0][1].r() );

    BOOST_CHECK_EQUAL( res[0].second[2].x(), refTrio[0][2].x() );
    BOOST_CHECK_EQUAL( res[0].second[2].y(), refTrio[0][2].y() );
    BOOST_CHECK_EQUAL( res[0].second[2].r(), refTrio[0][2].r() );
}


BOOST_AUTO_TEST_CASE( Test_Case_Reverse )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  5.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  2.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first[0].x(), dc3.x() );
    BOOST_CHECK_EQUAL( res[0].first[0].y(), dc3.y() );
    BOOST_CHECK_EQUAL( res[0].first[0].r(), dc3.r() );

    BOOST_CHECK_EQUAL( res[0].first[1].x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first[1].y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first[1].r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].first[2].x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first[2].y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first[2].r(), dc1.r() );


    BOOST_CHECK_EQUAL( res[0].second[0].x(), refTrio[0][0].x() );
    BOOST_CHECK_EQUAL( res[0].second[0].y(), refTrio[0][0].y() );
    BOOST_CHECK_EQUAL( res[0].second[0].r(), refTrio[0][0].r() );

    BOOST_CHECK_EQUAL( res[0].second[1].x(), refTrio[0][1].x() );
    BOOST_CHECK_EQUAL( res[0].second[1].y(), refTrio[0][1].y() );
    BOOST_CHECK_EQUAL( res[0].second[1].r(), refTrio[0][1].r() );

    BOOST_CHECK_EQUAL( res[0].second[2].x(), refTrio[0][2].x() );
    BOOST_CHECK_EQUAL( res[0].second[2].y(), refTrio[0][2].y() );
    BOOST_CHECK_EQUAL( res[0].second[2].r(), refTrio[0][2].r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Two_Points )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}


BOOST_AUTO_TEST_CASE( Test_Case_Radius_Pb )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.1 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Distance_Pb )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  3. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  2. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  3. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.2;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}


BOOST_AUTO_TEST_CASE( Test_Case_Perimeter_Pb )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.1 );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x( -0.2 );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.2  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.3;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Two_Circle )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.3;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_4_Points )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::DetectedCircle dc4;
    dc4.x(  1. );
    dc4.y(  5.  );
    dc4.r(  0.14 );
    detectedCircles.push_back( dc4 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.5;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first[0].x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first[0].y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first[0].r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].first[1].x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first[1].y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first[1].r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].first[2].x(), dc3.x() );
    BOOST_CHECK_EQUAL( res[0].first[2].y(), dc3.y() );
    BOOST_CHECK_EQUAL( res[0].first[2].r(), dc3.r() );


    BOOST_CHECK_EQUAL( res[0].second[0].x(), refTrio[0][0].x() );
    BOOST_CHECK_EQUAL( res[0].second[0].y(), refTrio[0][0].y() );
    BOOST_CHECK_EQUAL( res[0].second[0].r(), refTrio[0][0].r() );

    BOOST_CHECK_EQUAL( res[0].second[1].x(), refTrio[0][1].x() );
    BOOST_CHECK_EQUAL( res[0].second[1].y(), refTrio[0][1].y() );
    BOOST_CHECK_EQUAL( res[0].second[1].r(), refTrio[0][1].r() );

    BOOST_CHECK_EQUAL( res[0].second[2].x(), refTrio[0][2].x() );
    BOOST_CHECK_EQUAL( res[0].second[2].y(), refTrio[0][2].y() );
    BOOST_CHECK_EQUAL( res[0].second[2].r(), refTrio[0][2].r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_5_Points_associate )
{
    std::vector< std::vector<lsl::Circle> > refTrio;

    std::vector<lsl::Circle> triangle;
    triangle.push_back( lsl::Circle(1., 2., 0.04) );
    triangle.push_back( lsl::Circle(0., 2., 0.04) );
    triangle.push_back( lsl::Circle(1., 5., 0.04) );

    refTrio.push_back( triangle );

    std::vector< lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::DetectedCircle dc4;
    dc4.x(  1. );
    dc4.y(  5.1  );
    dc4.r(  0.04 );
    detectedCircles.push_back( dc4 );

    lsl::TrioCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.;
    p.maxLengthTolerance = 0.1;
    p.medLengthTolerance = 0.1;
    p.minLengthTolerance = 0.1;

    std::vector< std::pair< std::vector<lsl::DetectedCircle>, std::vector<lsl::Circle> > > res;
    res = lsl::TrioCircleIdentif::apply( detectedCircles, refTrio, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first[0].x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first[0].y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first[0].r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].first[1].x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first[1].y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first[1].r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].first[2].x(), dc3.x() );
    BOOST_CHECK_EQUAL( res[0].first[2].y(), dc3.y() );
    BOOST_CHECK_EQUAL( res[0].first[2].r(), dc3.r() );


    BOOST_CHECK_EQUAL( res[0].second[0].x(), refTrio[0][0].x() );
    BOOST_CHECK_EQUAL( res[0].second[0].y(), refTrio[0][0].y() );
    BOOST_CHECK_EQUAL( res[0].second[0].r(), refTrio[0][0].r() );

    BOOST_CHECK_EQUAL( res[0].second[1].x(), refTrio[0][1].x() );
    BOOST_CHECK_EQUAL( res[0].second[1].y(), refTrio[0][1].y() );
    BOOST_CHECK_EQUAL( res[0].second[1].r(), refTrio[0][1].r() );

    BOOST_CHECK_EQUAL( res[0].second[2].x(), refTrio[0][2].x() );
    BOOST_CHECK_EQUAL( res[0].second[2].y(), refTrio[0][2].y() );
    BOOST_CHECK_EQUAL( res[0].second[2].r(), refTrio[0][2].r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Associate_BadArguments )
{
    std::vector<lsl::Circle> refTrio;

    std::vector< lsl::DetectedCircle> mesTrio;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y(  2.  );
    dc1.r(  0.04 );
    mesTrio.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  0. );
    dc2.y(  2.  );
    dc2.r(  0.04 );
    mesTrio.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  1. );
    dc3.y(  5.  );
    dc3.r(  0.04 );
    mesTrio.push_back( dc3 );

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res;
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Associate_Quelconque )
{
    std::vector<lsl::Circle> refTrio;
    refTrio.push_back( lsl::Circle(0., 0., 0.04) );
    refTrio.push_back( lsl::Circle(1., 5., 0.04) );
    refTrio.push_back( lsl::Circle(2., 1., 0.04) );

    std::vector< lsl::DetectedCircle> mesTrio;
    lsl::DetectedCircle dcA, dcB, dcC;
    dcA.x( 0.  );   dcA.y( 0. );   dcA.r(  0.04 );
    dcB.x( 1.  );   dcB.y( 5. );   dcB.r(  0.04 );
    dcC.x( 2. );    dcC.y( 1.  );   dcC.r(  0.04 );

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res;

    mesTrio.clear();
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcC );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcB );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcA );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcC );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcA );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcB );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Associate_Isocele )
{
    std::vector<lsl::Circle> refTrio;
    refTrio.push_back( lsl::Circle(0., 0., 0.04) );
    refTrio.push_back( lsl::Circle(2., 0., 0.04) );
    refTrio.push_back( lsl::Circle(1., 4., 0.04) );

    std::vector< lsl::DetectedCircle> mesTrio;
    lsl::DetectedCircle dcA, dcB, dcC;
    dcA.x( 0. );   dcA.y( 0. );   dcA.r( 0.04 );
    dcB.x( 2. );   dcB.y( 0. );   dcB.r( 0.04 );
    dcC.x( 1. );   dcC.y( 4. );   dcC.r( 0.04 );

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res;

    mesTrio.clear();
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcC );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcB );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcA );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcC );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcA );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );

    mesTrio.clear();
    mesTrio.push_back( dcC );
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcB );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 3 );
    BOOST_CHECK_EQUAL( res[0].first.x(), dcA.x() );
//    BOOST_CHECK_EQUAL( res[0].first.y(), dcA.y() );
    BOOST_CHECK_EQUAL( res[1].first.x(), dcB.x() );
//    BOOST_CHECK_EQUAL( res[1].first.y(), dcB.y() );
    BOOST_CHECK_EQUAL( res[2].first.x(), dcC.x() );
//    BOOST_CHECK_EQUAL( res[2].first.y(), dcC.y() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Associate_Equilateral )
{
    std::vector<lsl::Circle> refTrio;
    refTrio.push_back( lsl::Circle(0., 0., 0.04) );
    refTrio.push_back( lsl::Circle(4., 0., 0.04) );
    refTrio.push_back( lsl::Circle(2., sqrt(12.)+0.01, 0.04) );

    std::vector< lsl::DetectedCircle> mesTrio;
    lsl::DetectedCircle dcA, dcB, dcC;
    dcA.x( 0. );   dcA.y( 0. );   dcA.r( 0.04 );
    dcB.x( 4. );   dcB.y( 0. );   dcB.r( 0.04 );
    dcC.x( 2. );   dcC.y( sqrt(12.) );   dcC.r( 0.04 );

    std::vector< std::pair<lsl::DetectedCircle, lsl::Circle> > res;

    mesTrio.clear();
    mesTrio.push_back( dcB );
    mesTrio.push_back( dcA );
    mesTrio.push_back( dcC );
    res = lsl::TrioCircleIdentif::associate( mesTrio, refTrio );
    BOOST_CHECK_EQUAL( res.size() , 0 );
}


BOOST_AUTO_TEST_SUITE_END()
