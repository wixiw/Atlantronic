/*
 * uTest_LaserOnlyLocalizator.hpp
 *
 *  Created on: 8 Mai 2012
 *      Author: boris
 */

#include "KFL/LaserOnlyLocalizator.hpp"

#include "LSL/tools/JsonScanParser.hpp"

#include "KFL/Logger.hpp"
#include <tools/vjson/JsonDocument.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace kfl;
using namespace arp_core::log;


BOOST_AUTO_TEST_SUITE( unittest_LaserOnlyLocalizator )

//BOOST_AUTO_TEST_CASE( estimateFromConstellation_BadArguments )
//{
//    std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdc;
//    arp_math::EstimatedPose2D pose;
//    bool res;
//    res =  LaserOnlyLocalizator::estimateFromConstellation(vpdc, pose);
//    BOOST_CHECK( !res );
//
//    vpdc.push_back( std::make_pair(lsl::DetectedCircle(), lsl::Circle()) );
//    LaserOnlyLocalizator obj;
//    res =  obj.estimateFromConstellation(vpdc, resPose);
//    BOOST_CHECK( !res );
//
//}
//
//BOOST_AUTO_TEST_CASE( estimateFromConstellation_Triangle )
//{
//    std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdc;
//
//    arp_math::Pose2D H(0.5, -0.1, M_PI/4.);
//
//    lsl::Circle c1;
//    c1.x(1.5);      c1.y(0.);       c1.r(0.04);
//    Vector2 dc1Pos = H.inverse() * Vector2(c1.x(), c1.y());
//    lsl::DetectedCircle dc1;
//    dc1.x(dc1Pos(0));  dc1.y(dc1Pos(1));  dc1.r(c1.r());
//    vpdc.push_back( std::make_pair( dc1, c1 ) );
//
//    lsl::Circle c2;
//    c2.x(-1.5);      c2.y(1.);      c2.r(0.04);
//    Vector2 dc2Pos = H.inverse() * Vector2(c2.x(), c2.y());
//    lsl::DetectedCircle dc2;
//    dc2.x(dc2Pos(0));  dc2.y(dc2Pos(1));  dc2.r(c2.r());
//    vpdc.push_back( std::make_pair( dc2, c2 ) );
//
//    lsl::Circle c3;
//    c3.x(-1.5);      c3.y(-1.);     c3.r(0.04);
//    Vector2 dc3Pos = H.inverse() * Vector2(c3.x(), c3.y());
//    lsl::DetectedCircle dc3;
//    dc3.x(dc3Pos(0));  dc3.y(dc3Pos(1));  dc3.r(c3.r());
//    vpdc.push_back( std::make_pair( dc3, c3 ) );
//
//
//    arp_math::EstimatedPose2D resPose;
//    bool res;
//    LaserOnlyLocalizator obj;
//    res =  obj.estimateFromConstellation(vpdc, resPose);
//    BOOST_CHECK( res );
//
//    BOOST_CHECK_CLOSE( resPose.x(), H.x(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.y(), H.y(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.h(), H.h(), 1.f );
//
//}
//
//BOOST_AUTO_TEST_CASE( estimateFromConstellation_Quadrangle )
//{
//    std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdc;
//
//    arp_math::Pose2D H(0.5, -0.1, M_PI/4.);
//
//    lsl::Circle c1;
//    c1.x(1.5);      c1.y(0.);       c1.r(0.04);
//    Vector2 dc1Pos = H.inverse() * Vector2(c1.x(), c1.y());
//    lsl::DetectedCircle dc1;
//    dc1.x(dc1Pos(0));  dc1.y(dc1Pos(1));  dc1.r(c1.r());
//    vpdc.push_back( std::make_pair( dc1, c1 ) );
//
//    lsl::Circle c2;
//    c2.x(-1.5);      c2.y(1.);      c2.r(0.04);
//    Vector2 dc2Pos = H.inverse() * Vector2(c2.x(), c2.y());
//    lsl::DetectedCircle dc2;
//    dc2.x(dc2Pos(0));  dc2.y(dc2Pos(1));  dc2.r(c2.r());
//    vpdc.push_back( std::make_pair( dc2, c2 ) );
//
//    lsl::Circle c3;
//    c3.x(-1.5);      c3.y(-1.);     c3.r(0.04);
//    Vector2 dc3Pos = H.inverse() * Vector2(c3.x(), c3.y());
//    lsl::DetectedCircle dc3;
//    dc3.x(dc3Pos(0));  dc3.y(dc3Pos(1));  dc3.r(c3.r());
//    vpdc.push_back( std::make_pair( dc3, c3 ) );
//
//    lsl::Circle c4;
//    c4.x(0.);      c4.y(0.);     c4.r(0.02);
//    Vector2 dc4Pos = H.inverse() * Vector2(c4.x(), c4.y());
//    lsl::DetectedCircle dc4;
//    dc4.x(dc4Pos(0));  dc4.y(dc4Pos(1));  dc4.r(c4.r());
//    vpdc.push_back( std::make_pair( dc4, c4 ) );
//
//
//    arp_math::EstimatedPose2D resPose;
//    bool res;
//    LaserOnlyLocalizator obj;
//    res =  obj.estimateFromConstellation(vpdc, resPose);
//    BOOST_CHECK( res );
//
//    BOOST_CHECK_CLOSE( resPose.x(), H.x(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.y(), H.y(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.h(), H.h(), 1.f );
//
//}
//
//BOOST_AUTO_TEST_CASE( estimateFromConstellation_Segment_1 )
//{
//    std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdc;
//
//    arp_math::Pose2D H(0.5, -0.1, M_PI/4.);
//
//    lsl::Circle c1;
//    c1.x(-1.5);      c1.y(1.);      c1.r(0.04);
//    Vector2 dc1Pos = H.inverse() * Vector2(c1.x(), c1.y());
//    lsl::DetectedCircle dc1;
//    dc1.x(dc1Pos(0));  dc1.y(dc1Pos(1));  dc1.r(c1.r());
//    vpdc.push_back( std::make_pair( dc1, c1 ) );
//
//    lsl::Circle c2;
//    c2.x(-1.5);      c2.y(-1.);     c2.r(0.04);
//    Vector2 dc2Pos = H.inverse() * Vector2(c2.x(), c2.y());
//    lsl::DetectedCircle dc2;
//    dc2.x(dc2Pos(0));  dc2.y(dc2Pos(1));  dc2.r(c2.r());
//    vpdc.push_back( std::make_pair( dc2, c2 ) );
//
//    arp_math::EstimatedPose2D resPose;
//    bool res;
//    LaserOnlyLocalizator obj;
//    res =  obj.estimateFromConstellation(vpdc, resPose);
//    BOOST_CHECK( res );
//
//    BOOST_CHECK_CLOSE( resPose.x(), H.x(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.y(), H.y(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.h(), H.h(), 1.f );
//}
//
//BOOST_AUTO_TEST_CASE( estimateFromConstellation_Segment_2 )
//{
//    std::vector< std::pair< lsl::DetectedCircle, lsl::Circle > > vpdc;
//
//    arp_math::Pose2D H(0.8, -0.2, -M_PI/8.);
//
//    lsl::Circle c1;
//    c1.x( 1.5);      c1.y(0.);      c1.r(0.04);
//    Vector2 dc1Pos = H.inverse() * Vector2(c1.x(), c1.y());
//    lsl::DetectedCircle dc1;
//    dc1.x(dc1Pos(0));  dc1.y(dc1Pos(1));  dc1.r(c1.r());
//    vpdc.push_back( std::make_pair( dc1, c1 ) );
//
//    lsl::Circle c2;
//    c2.x(-1.5);      c2.y(-1.);     c2.r(0.04);
//    Vector2 dc2Pos = H.inverse() * Vector2(c2.x(), c2.y());
//    lsl::DetectedCircle dc2;
//    dc2.x(dc2Pos(0));  dc2.y(dc2Pos(1));  dc2.r(c2.r());
//    vpdc.push_back( std::make_pair( dc2, c2 ) );
//
//    arp_math::EstimatedPose2D resPose;
//    bool res;
//    LaserOnlyLocalizator obj;
//    res =  obj.estimateFromConstellation(vpdc, resPose);
//    BOOST_CHECK( res );
//
//    BOOST_CHECK_CLOSE( resPose.x(), H.x(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.y(), H.y(), 1.f );
//    BOOST_CHECK_CLOSE( resPose.h(), H.h(), 1.f );
//}

namespace unittest_LaserOnlyLocalizator
{
    enum TYPE
    {
        NONE = 0,
        BIG_SEGMENT_AMBIGUOUS = 1,
        BIG_SEGMENT_EASY = 2,
        SMALL_SEGMENT = 3,
        TRIANGLE = 4
    };

    void doTest(unsigned int xpIndex, TYPE type)
    {
        double transPrecision = 0.010;
        double rotPrecision = deg2rad(1.0);

        std::string p = ros::package::getPath("arp_rlu");

        std::stringstream xpSS;
        xpSS << xpIndex;
        std::string xpName = xpSS.str();

        std::string scanFileName = p + "/ressource/unittest/KFL/LaserOnlyLocalizator/laseronlyloc_" + xpName + "/scan.json";
        arp_rlu::lsl::JsonScanParser parser( scanFileName.c_str() );
        lsl::LaserScan ls;
        bool res = parser.getScan(ls);
        BOOST_CHECK(res);

        kfl::Log( DEBUG ) << "Scan size : " << ls.getSize();

        kfl::LaserOnlyLocalizator obj;

        kfl::LaserOnlyLocalizator::Params     procParams;
        procParams.mfp.width = 3;
        procParams.pcp.minRange = 0.01;
        procParams.pcp.maxRange = 10.0;
        procParams.pcp.minTheta = -PI;
        procParams.pcp.maxTheta = PI;
        procParams.psp.rangeThres = 0.08;
        procParams.minNbPoints = 4;
        procParams.cip.radius = 0.04;
        procParams.cip.coeffs = std::vector<double>();
        procParams.cip.coeffs.push_back( 1.0);
        procParams.cip.coeffs.push_back( 0.034);
        procParams.tcp.radiusTolerance = 0.03;
        procParams.tcp.distanceTolerance = 0.6;
        procParams.tcp.maxLengthTolerance = 0.05;
        procParams.tcp.medLengthTolerance = 0.05;
        procParams.tcp.minLengthTolerance = 0.05;
        procParams.referencedBeacons = std::vector<lsl::Circle>();
        procParams.referencedBeacons.push_back(lsl::Circle( 1.560, 0.0, 0.04));
        procParams.referencedBeacons.push_back(lsl::Circle(-1.555, 1.040, 0.04));
        procParams.referencedBeacons.push_back(lsl::Circle(-1.555,-1.040, 0.04));

        obj.setParams(procParams);


        BOOST_CHECK( obj.process(ls) == (type >= BIG_SEGMENT_EASY) );
        if( type >= BIG_SEGMENT_EASY )
        {
            vjson::JsonDocument docResults;
            std::string resultsFileName = p + "/ressource/unittest/KFL/LaserOnlyLocalizator/laseronlyloc_" + xpName + "/position.json";
            BOOST_CHECK( docResults.parse( resultsFileName.c_str() ) );

            double x = docResults.getFloatData( docResults.getChild( docResults.root(), "x") );
            double y = docResults.getFloatData( docResults.getChild( docResults.root(), "y") );
            double h = docResults.getFloatData( docResults.getChild( docResults.root(), "h") );

            arp_math::EstimatedPose2D pose = obj.getEstimatedPose();

            BOOST_CHECK_SMALL( pose.x() - x , transPrecision );
            BOOST_CHECK_SMALL( pose.y() - y , transPrecision );
            BOOST_CHECK_SMALL( betweenMinusPiAndPlusPi(pose.h()- h), rotPrecision );

        }

        kfl::Log( NOTICE ) << "\n";
        kfl::Log( NOTICE ) << "**************** Experience " << xpName << " *******************************";
        kfl::Log( NOTICE ) << obj.getPerformanceReport();
        kfl::Log( NOTICE ) << "************************************************************";
        kfl::Log( NOTICE ) << "\n";
    }
}

BOOST_AUTO_TEST_CASE( test_0 )
{
    unittest_LaserOnlyLocalizator::doTest(0, unittest_LaserOnlyLocalizator::TRIANGLE);
}

BOOST_AUTO_TEST_CASE( test_1 )
{
    unittest_LaserOnlyLocalizator::doTest(1, unittest_LaserOnlyLocalizator::TRIANGLE);
}

BOOST_AUTO_TEST_CASE( test_2 )
{
    unittest_LaserOnlyLocalizator::doTest(2, unittest_LaserOnlyLocalizator::BIG_SEGMENT_EASY);
}

BOOST_AUTO_TEST_CASE( test_3 )
{
    unittest_LaserOnlyLocalizator::doTest(3, unittest_LaserOnlyLocalizator::TRIANGLE);
}

BOOST_AUTO_TEST_CASE( test_4 )
{
    unittest_LaserOnlyLocalizator::doTest(4, unittest_LaserOnlyLocalizator::BIG_SEGMENT_EASY);
}

BOOST_AUTO_TEST_CASE( test_5 )
{
    unittest_LaserOnlyLocalizator::doTest(5, unittest_LaserOnlyLocalizator::SMALL_SEGMENT);
}

BOOST_AUTO_TEST_CASE( test_6 )
{
    unittest_LaserOnlyLocalizator::doTest(6, unittest_LaserOnlyLocalizator::NONE);
}

BOOST_AUTO_TEST_CASE( test_7 )
{
    unittest_LaserOnlyLocalizator::doTest(7, unittest_LaserOnlyLocalizator::NONE);
}

BOOST_AUTO_TEST_CASE( test_8 )
{
    unittest_LaserOnlyLocalizator::doTest(8, unittest_LaserOnlyLocalizator::SMALL_SEGMENT);
}

BOOST_AUTO_TEST_CASE( test_9 )
{
    unittest_LaserOnlyLocalizator::doTest(9, unittest_LaserOnlyLocalizator::BIG_SEGMENT_EASY);
}

BOOST_AUTO_TEST_CASE( test_10 )
{
    unittest_LaserOnlyLocalizator::doTest(10, unittest_LaserOnlyLocalizator::SMALL_SEGMENT);
}

BOOST_AUTO_TEST_SUITE_END()
