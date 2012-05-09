/*
 * uTest_DuoCircleIdentif.hpp
 *
 *  Created on: 23 Mars 2012
 *      Author: boris
 */

#include "LSL/identificators/DuoCircleIdentif.hpp"

using namespace arp_math;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_SUITE( unittest_DuoCircleIdentif )

BOOST_AUTO_TEST_CASE( Test_checkConsistency )
{
    lsl::DuoCircleIdentif::Params p;
    BOOST_CHECK( p.checkConsistency() );

    p.radiusTolerance = 0.1;
    p.distanceTolerance = 0.6;
    p.lengthTolerance = 0.2;
    BOOST_CHECK( p.checkConsistency() );

    p.radiusTolerance = -0.1;
    p.distanceTolerance = 0.6;
    p.lengthTolerance = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = -0.6;
    p.lengthTolerance = 0.2;
    BOOST_CHECK( !(p.checkConsistency()) );

    p.radiusTolerance =  0.1;
    p.distanceTolerance = 0.6;
    p.lengthTolerance = -0.2;
    BOOST_CHECK( !(p.checkConsistency()) );
}

BOOST_AUTO_TEST_CASE( Test_Case_Empty )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;
    std::vector< lsl::DetectedCircle> detectedCircles;
    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Trivial )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y( -2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  5. );
    dc2.y( -2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].second.first.x(), refDuo[0].first.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), refDuo[0].first.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), refDuo[0].first.r() );

    BOOST_CHECK_EQUAL( res[0].second.second.x(), refDuo[0].second.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), refDuo[0].second.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), refDuo[0].second.r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Trivial_Reverse )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  5. );
    dc1.y( -2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  1. );
    dc2.y( -2.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc1.r() );

    BOOST_CHECK_EQUAL( res[0].second.first.x(), refDuo[0].first.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), refDuo[0].first.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), refDuo[0].first.r() );

    BOOST_CHECK_EQUAL( res[0].second.second.x(), refDuo[0].second.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), refDuo[0].second.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), refDuo[0].second.r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_Radius_Pb )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.   );
    dc1.y( -2.   );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  5.   );
    dc2.y( -2.   );
    dc2.r(  0.12 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Distance_Pb_1 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.   );
    dc1.y( -2.   );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  1.   );
    dc2.y( -6.   );
    dc2.r(  0.05 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Distance_Pb_2 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.   );
    dc1.y( -6.   );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  1.   );
    dc2.y( -2.   );
    dc2.r(  0.05 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}


BOOST_AUTO_TEST_CASE( Test_Case_Length_Pb_2 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1.1   );
    dc1.y( -2.   );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x(  4.9   );
    dc2.y( -2.   );
    dc2.r(  0.05 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.3;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 0 );
}

BOOST_AUTO_TEST_CASE( Test_Case_Complex )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    refDuo.push_back( std::make_pair( lsl::Circle(1., -2., 0.04), lsl::Circle(5., -2., 0.05) ));
    refDuo.push_back( std::make_pair( lsl::Circle(-2., 0., 0.15), lsl::Circle(-2., 4., 0.15) ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x(  1. );
    dc1.y( -2.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x( -2. );
    dc2.y(  4.  );
    dc2.r(  0.15 );
    detectedCircles.push_back( dc2 );

    lsl::DetectedCircle dc3;
    dc3.x(  5. );
    dc3.y( -2.  );
    dc3.r(  0.04 );
    detectedCircles.push_back( dc3 );

    lsl::DetectedCircle dc4;
    dc4.x( -2. );
    dc4.y(  0.  );
    dc4.r(  0.16 );
    detectedCircles.push_back( dc4 );

    lsl::DetectedCircle dc5;
    dc5.x( -2.   );
    dc5.y(  0.1  );
    dc5.r(  0.16 );
    detectedCircles.push_back( dc5 );


    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.1;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 2 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc1.r() );
    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc3.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc3.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc3.r() );

    BOOST_CHECK_EQUAL( res[0].second.first.x(), refDuo[0].first.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), refDuo[0].first.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), refDuo[0].first.r() );
    BOOST_CHECK_EQUAL( res[0].second.second.x(), refDuo[0].second.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), refDuo[0].second.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), refDuo[0].second.r() );


    BOOST_CHECK_EQUAL( res[1].first.first.x(), dc4.x() );
    BOOST_CHECK_EQUAL( res[1].first.first.y(), dc4.y() );
    BOOST_CHECK_EQUAL( res[1].first.first.r(), dc4.r() );
    BOOST_CHECK_EQUAL( res[1].first.second.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[1].first.second.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[1].first.second.r(), dc2.r() );

    BOOST_CHECK_EQUAL( res[1].second.first.x(), refDuo[1].first.x() );
    BOOST_CHECK_EQUAL( res[1].second.first.y(), refDuo[1].first.y() );
    BOOST_CHECK_EQUAL( res[1].second.first.r(), refDuo[1].first.r() );
    BOOST_CHECK_EQUAL( res[1].second.second.x(), refDuo[1].second.x() );
    BOOST_CHECK_EQUAL( res[1].second.second.y(), refDuo[1].second.y() );
    BOOST_CHECK_EQUAL( res[1].second.second.r(), refDuo[1].second.r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_association_1 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    lsl::Circle c1(-1.5, 1., 0.04);
    lsl::Circle c2(-1.5,-1., 0.04);
    refDuo.push_back( std::make_pair( c1, c2 ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x( -1.5 );
    dc1.y(  1.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x( -1.5 );
    dc2.y( -1.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc1.r() );
    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc2.r() );
    BOOST_CHECK_EQUAL( res[0].second.first.x(), c1.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), c1.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), c1.r() );
    BOOST_CHECK_EQUAL( res[0].second.second.x(), c2.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), c2.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), c2.r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_association_2 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    lsl::Circle c1(-1.5, 1., 0.04);
    lsl::Circle c2(-1.5,-1., 0.04);
    refDuo.push_back( std::make_pair( c1, c2 ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x( -1.5 );
    dc1.y( -1.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x( -1.5 );
    dc2.y(  1.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc2.r() );
    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc1.r() );
    BOOST_CHECK_EQUAL( res[0].second.first.x(), c1.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), c1.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), c1.r() );
    BOOST_CHECK_EQUAL( res[0].second.second.x(), c2.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), c2.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), c2.r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_association_3 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    lsl::Circle c1(-1.5,-1., 0.04);
    lsl::Circle c2(-1.5, 1., 0.04);
    refDuo.push_back( std::make_pair( c1, c2 ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x( -1.5 );
    dc1.y(  1.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x( -1.5 );
    dc2.y( -1.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc1.r() );
    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc2.r() );
    BOOST_CHECK_EQUAL( res[0].second.first.x(), c2.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), c2.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), c2.r() );
    BOOST_CHECK_EQUAL( res[0].second.second.x(), c1.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), c1.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), c1.r() );
}

BOOST_AUTO_TEST_CASE( Test_Case_association_4 )
{
    std::vector< std::pair<lsl::Circle, lsl::Circle> > refDuo;

    lsl::Circle c1(-1.5,-1., 0.04);
    lsl::Circle c2(-1.5, 1., 0.04);
    refDuo.push_back( std::make_pair( c1, c2 ));

    std::vector<lsl::DetectedCircle> detectedCircles;

    lsl::DetectedCircle dc1;
    dc1.x( -1.5 );
    dc1.y( -1.  );
    dc1.r(  0.04 );
    detectedCircles.push_back( dc1 );

    lsl::DetectedCircle dc2;
    dc2.x( -1.5 );
    dc2.y(  1.  );
    dc2.r(  0.04 );
    detectedCircles.push_back( dc2 );

    lsl::DuoCircleIdentif::Params p;
    p.radiusTolerance = 0.05;
    p.distanceTolerance = 0.;
    p.lengthTolerance = 0.1;

    std::vector< std::pair< std::pair<lsl::DetectedCircle, lsl::DetectedCircle>, std::pair<lsl::Circle, lsl::Circle> > > res;
    res = lsl::DuoCircleIdentif::apply( detectedCircles, refDuo, p);

    BOOST_CHECK_EQUAL( res.size() , 1 );

    BOOST_CHECK_EQUAL( res[0].first.first.x(), dc2.x() );
    BOOST_CHECK_EQUAL( res[0].first.first.y(), dc2.y() );
    BOOST_CHECK_EQUAL( res[0].first.first.r(), dc2.r() );
    BOOST_CHECK_EQUAL( res[0].first.second.x(), dc1.x() );
    BOOST_CHECK_EQUAL( res[0].first.second.y(), dc1.y() );
    BOOST_CHECK_EQUAL( res[0].first.second.r(), dc1.r() );
    BOOST_CHECK_EQUAL( res[0].second.first.x(), c2.x() );
    BOOST_CHECK_EQUAL( res[0].second.first.y(), c2.y() );
    BOOST_CHECK_EQUAL( res[0].second.first.r(), c2.r() );
    BOOST_CHECK_EQUAL( res[0].second.second.x(), c1.x() );
    BOOST_CHECK_EQUAL( res[0].second.second.y(), c1.y() );
    BOOST_CHECK_EQUAL( res[0].second.second.r(), c1.r() );
}

BOOST_AUTO_TEST_SUITE_END()
