/*
 * uTest_BeaconDetector_deprecated.hpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */

#include "KFL/BeaconDetector_deprecated.hpp"

#include "LSL/tools/JsonScanParser.hpp"

#include "KFL/Logger.hpp"
#include <tools/vjson/JsonDocument.hpp>

using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace lsl;
using namespace kfl;
using namespace arp_core::log;


BOOST_AUTO_TEST_SUITE( unittest_BeaconDetector_deprecated )


BOOST_AUTO_TEST_CASE( load_traj_1 )
{
    vjson::JsonDocument docTraj;
    BOOST_CHECK( docTraj.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/traj_1.json") );
    unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
    Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
        xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
        yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
        hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
    }
}

BOOST_AUTO_TEST_CASE( load_scan_1 )
{
    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/KFL/BeaconDetector_deprecated/scan_1.json");
    lsl::LaserScan ls;
    bool res = parser.getScan(ls);
    BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE( load_results_1 )
{
    vjson::JsonDocument docResults;
    BOOST_CHECK( docResults.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/results_1.json") );
}

BOOST_AUTO_TEST_CASE( test_DetectedCircle_1 )
{
    vjson::JsonDocument docTraj;
    BOOST_CHECK( docTraj.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/traj_1.json") );
    unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
    Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
        xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
        yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
        hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
    }

    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/KFL/BeaconDetector_deprecated/scan_1.json");
    lsl::LaserScan ls;
    bool res = parser.getScan(ls);
    BOOST_CHECK(res);
    unsigned int N = ls.getSize();

    BOOST_CHECK_EQUAL(n , N);

    kfl::BeaconDetector_deprecated obj;
    obj.process(ls, tt, xx, yy, hh);

    std::vector< DetectedCircle > vdc = obj.getDetectedCircles();

    vjson::JsonDocument docResults;
    BOOST_CHECK( docResults.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/results_1.json") );

    unsigned int nbObjects = docResults.getIntegerData( docResults.getChild( docResults.root(), "nbObjects") );

    BOOST_CHECK_EQUAL( nbObjects, vdc.size() );

    for(unsigned int i = 0 ; i < nbObjects ; i++)
    {
        std::stringstream ss;
        ss << "obj_" << i;
        double xCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "xCenter") );
        double yCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "yCenter") );

        BOOST_CHECK_CLOSE( vdc[i].r() , 0.04, 1.f);
        BOOST_CHECK( sqrt( (vdc[i].x()-xCenter)*(vdc[i].x()-xCenter) + (vdc[i].y()-yCenter)*(vdc[i].y()-yCenter) ) < 0.025);
    }
}

BOOST_AUTO_TEST_CASE( test_DetectedCircle_2 )
{
    vjson::JsonDocument docTraj;
    BOOST_CHECK( docTraj.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/traj_2.json") );
    unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
    Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
        xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
        yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
        hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
    }

    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/KFL/BeaconDetector_deprecated/scan_2.json");
    lsl::LaserScan ls;
    bool res = parser.getScan(ls);
    BOOST_CHECK(res);
    unsigned int N = ls.getSize();

    BOOST_CHECK_EQUAL(n , N);

    kfl::BeaconDetector_deprecated obj;
    obj.process(ls, tt, xx, yy, hh);

    std::vector< DetectedCircle > vdc = obj.getDetectedCircles();

    vjson::JsonDocument docResults;
    BOOST_CHECK( docResults.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/results_2.json") );

    unsigned int nbObjects = docResults.getIntegerData( docResults.getChild( docResults.root(), "nbObjects") );

    BOOST_CHECK_EQUAL( nbObjects, vdc.size() );
    Log( DEBUG ) << "test_DetectedCircle_2" << " - " << "vdc.size()=" << vdc.size();

    for(unsigned int i = 0 ; i < nbObjects ; i++)
    {
        std::stringstream ss;
        ss << "obj_" << i;
        double xCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "xCenter") );
        double yCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "yCenter") );

        Log( DEBUG ) << "test_DetectedCircle_2" << " - " << "   dc[" << i << "]  xCenter=" << xCenter << "  yCenter=" << yCenter << "   vdc[i].x()=" << vdc[i].x() << "   vdc[i].y()=" << vdc[i].y();

        BOOST_CHECK_CLOSE( vdc[i].r() , 0.04, 1.f);
        BOOST_CHECK( sqrt( (vdc[i].x()-xCenter)*(vdc[i].x()-xCenter) + (vdc[i].y()-yCenter)*(vdc[i].y()-yCenter) ) < 0.025);
    }
}

BOOST_AUTO_TEST_CASE( test_DetectedCircle_3 )
{
    vjson::JsonDocument docTraj;
    BOOST_CHECK( docTraj.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/traj_3.json") );
    unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
    Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
        xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
        yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
        hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
    }

    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/KFL/BeaconDetector_deprecated/scan_3.json");
    lsl::LaserScan ls;
    bool res = parser.getScan(ls);
    BOOST_CHECK(res);
    unsigned int N = ls.getSize();

    BOOST_CHECK_EQUAL(n , N);

    kfl::BeaconDetector_deprecated obj;
    obj.process(ls, tt, xx, yy, hh);

    std::vector< DetectedCircle > vdc = obj.getDetectedCircles();

    vjson::JsonDocument docResults;
    BOOST_CHECK( docResults.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/results_3.json") );

    unsigned int nbObjects = docResults.getIntegerData( docResults.getChild( docResults.root(), "nbObjects") );

    BOOST_CHECK_EQUAL( nbObjects, vdc.size() );
    Log( DEBUG ) << "test_DetectedCircle_2" << " - " << "vdc.size()=" << vdc.size();

    for(unsigned int i = 0 ; i < nbObjects ; i++)
    {
        std::stringstream ss;
        ss << "obj_" << i;
        double xCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "xCenter") );
        double yCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "yCenter") );

        Log( DEBUG ) << "test_DetectedCircle_3" << " - " << "   dc[" << i << "]  xCenter=" << xCenter << "  yCenter=" << yCenter << "   vdc[i].x()=" << vdc[i].x() << "   vdc[i].y()=" << vdc[i].y();

        BOOST_CHECK_CLOSE( vdc[i].r() , 0.04, 1.f);
        BOOST_CHECK( sqrt( (vdc[i].x()-xCenter)*(vdc[i].x()-xCenter) + (vdc[i].y()-yCenter)*(vdc[i].y()-yCenter) ) < 0.025);
    }
}

BOOST_AUTO_TEST_CASE( test_DetectedCircle_4 )
{
    vjson::JsonDocument docTraj;
    BOOST_CHECK( docTraj.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/traj_4.json") );
    unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
    Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
        xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
        yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
        hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
    }

    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/KFL/BeaconDetector_deprecated/scan_4.json");
    lsl::LaserScan ls;
    bool res = parser.getScan(ls);
    BOOST_CHECK(res);
    unsigned int N = ls.getSize();

    BOOST_CHECK_EQUAL(n , N);

    kfl::BeaconDetector_deprecated obj;
    std::vector<lsl::Circle> referencedBeacons;
    referencedBeacons.push_back(lsl::Circle(-1.5, 1.0, 0.04));
    referencedBeacons.push_back(lsl::Circle(-1.5,-1.0, 0.04));
    referencedBeacons.push_back(lsl::Circle( 1.5, 0.0, 0.04));
    obj.setReferencedBeacons(referencedBeacons);

    obj.process(ls, tt, xx, yy, hh);


    std::vector< DetectedCircle > vdc = obj.getDetectedCircles();

    vjson::JsonDocument docResults;
    BOOST_CHECK( docResults.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/results_4.json") );

    unsigned int nbObjects = docResults.getIntegerData( docResults.getChild( docResults.root(), "nbObjects") );

    BOOST_CHECK_EQUAL( nbObjects, vdc.size() );
    Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "vdc.size()=" << vdc.size();

    for(unsigned int i = 0 ; i < nbObjects ; i++)
    {
        std::stringstream ss;
        ss << "obj_" << i;
        double xCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "xCenter") );
        double yCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "yCenter") );

        Log( DEBUG ) << "test_DetectedCircle_4" << " - "
                << "   dc[" << i << "]"
                << "  xCenter=" << xCenter
                << "  yCenter=" << yCenter
                << "  vdc[i].x()=" << vdc[i].x()
                << "  vdc[i].y()=" << vdc[i].y()
                << "  vdc[i].t()=" << vdc[i].getApparentCenterTime();

        BOOST_CHECK_CLOSE( vdc[i].r() , 0.04, 1.f);
        BOOST_CHECK( sqrt( (vdc[i].x()-xCenter)*(vdc[i].x()-xCenter) + (vdc[i].y()-yCenter)*(vdc[i].y()-yCenter) ) < 0.025);
    }

    std::vector< std::pair<lsl::Circle, Eigen::Vector2d > > measures;
    Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "measures vector created";
    Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "tt.size()=" << tt.size();
    for(int i = 0 ; i < tt.size() ; i++)
    {
        lsl::Circle target;
        Eigen::Vector2d meas = Eigen::Vector2d::Zero();
        if( obj.getBeacon(tt[i], target, meas) )
        {
            measures.push_back( std::make_pair(target, meas) );
            Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "measure[" << measures.size() << "]:";
            Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * xTarget=" << target.x();
            Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * yTarget=" << target.y();
            Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * range=" << meas(0);
            Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * theta=" << meas(1);
        }
    }
    Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "measures done (measures.size()=" << measures.size() << ")";


    vjson::JsonDocument docMeas;
    BOOST_CHECK( docMeas.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/meas_4.json") );

    unsigned int nbMeas = docMeas.getIntegerData( docMeas.getChild( docMeas.root(), "nbMeas") );
    BOOST_CHECK_EQUAL(measures.size(), nbMeas);

    Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "results comparison";

    for(unsigned int i = 0 ; i < nbMeas ; i++)
    {
        Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "measures[" << i << "]:";

        std::stringstream ss;
        ss << "meas_" << i;
        double trueXBeacon = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "xBeacon") );
        double trueYBeacon = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "yBeacon") );
        double trueRange   = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "range") );
        double trueTheta   = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "theta") );

        Circle target = measures[i].first;
        Eigen::Vector2d meas = measures[i].second;

        Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * xTarget=" << target.x() << "   vs trueXBeacon=" << trueXBeacon;
        Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * yTarget=" << target.y() << "   vs trueYBeacon=" << trueYBeacon;
        Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * range="   << meas(0)    << "   vs trueRange="   << trueRange;
        Log( DEBUG ) << "test_DetectedCircle_4" << " - " << "  * theta="   << meas(1)    << "   vs trueTheta="   << trueTheta;


        BOOST_CHECK_CLOSE( target.x(), trueXBeacon, 1.f);
        BOOST_CHECK_CLOSE( target.y(), trueYBeacon, 1.f);
        BOOST_CHECK( abs(meas(0) - trueRange) < 0.01 );
        BOOST_CHECK( abs(meas(1) - trueTheta) < 0.01);
    }
}

BOOST_AUTO_TEST_CASE( test_DetectedCircle_5 )
{
    vjson::JsonDocument docTraj;
    BOOST_CHECK( docTraj.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/traj_5.json") );
    unsigned int n = docTraj.getIntegerData( docTraj.getChild( docTraj.root(), "size") );
    Eigen::VectorXd tt = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd xx = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd yy = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hh = Eigen::VectorXd::Zero(n);
    for(unsigned int i = 0 ; i < n ; i++)
    {
        tt[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "tt") , i ) );
        xx[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "xx") , i ) );
        yy[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "yy") , i ) );
        hh[i] = docTraj.getFloatData( docTraj.getChild( docTraj.getChild(docTraj.root(), "hh") , i ) );
    }

    arp_rlu::lsl::JsonScanParser parser("../ressource/unittest/KFL/BeaconDetector_deprecated/scan_5.json");
    lsl::LaserScan ls;
    bool res = parser.getScan(ls);
    BOOST_CHECK(res);
    unsigned int N = ls.getSize();

    BOOST_CHECK_EQUAL(n , N);

    kfl::BeaconDetector_deprecated obj;
    std::vector<lsl::Circle> referencedBeacons;
    referencedBeacons.push_back(lsl::Circle(-1.5, 1.0, 0.04));
    referencedBeacons.push_back(lsl::Circle(-1.5,-1.0, 0.04));
    referencedBeacons.push_back(lsl::Circle( 1.5, 0.0, 0.04));
    obj.setReferencedBeacons(referencedBeacons);

    obj.process(ls, tt, xx, yy, hh);


    std::vector< DetectedCircle > vdc = obj.getDetectedCircles();

    vjson::JsonDocument docResults;
    BOOST_CHECK( docResults.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/results_5.json") );

    unsigned int nbObjects = docResults.getIntegerData( docResults.getChild( docResults.root(), "nbObjects") );

    BOOST_CHECK_EQUAL( vdc.size(), nbObjects );
    Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "vdc.size()=" << vdc.size();

    for(unsigned int i = 0 ; i < nbObjects ; i++)
    {
        std::stringstream ss;
        ss << "obj_" << i;
        double xCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "xCenter") );
        double yCenter = docResults.getFloatData( docResults.getChild( docResults.getChild(docResults.root(), ss.str()), "yCenter") );

        Log( DEBUG ) << "test_DetectedCircle_5" << " - "
                << "   dc[" << i << "]"
                << "  xCenter=" << xCenter
                << "  yCenter=" << yCenter
                << "  vdc[i].x()=" << vdc[i].x()
                << "  vdc[i].y()=" << vdc[i].y()
                << "  vdc[i].t()=" << vdc[i].getApparentCenterTime();

        BOOST_CHECK_CLOSE( vdc[i].r() , 0.04, 1.f);
        BOOST_CHECK( sqrt( (vdc[i].x()-xCenter)*(vdc[i].x()-xCenter) + (vdc[i].y()-yCenter)*(vdc[i].y()-yCenter) ) < 0.025);
    }

    std::vector< std::pair<lsl::Circle, Eigen::Vector2d > > measures;
    Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "measures vector created";
    Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "tt.size()=" << tt.size();
    for(int i = 0 ; i < tt.size() ; i++)
    {
        lsl::Circle target;
        Eigen::Vector2d meas = Eigen::Vector2d::Zero();
        if( obj.getBeacon(tt[i], target, meas) )
        {
            measures.push_back( std::make_pair(target, meas) );
            Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "measure[" << measures.size() << "]:";
            Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * xTarget=" << target.x();
            Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * yTarget=" << target.y();
            Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * range=" << meas(0);
            Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * theta=" << meas(1);
        }
    }
    Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "measures done (measures.size()=" << measures.size() << ")";


    vjson::JsonDocument docMeas;
    BOOST_CHECK( docMeas.parse("../ressource/unittest/KFL/BeaconDetector_deprecated/meas_5.json") );

    unsigned int nbMeas = docMeas.getIntegerData( docMeas.getChild( docMeas.root(), "nbMeas") );
    BOOST_CHECK_EQUAL(measures.size(), nbMeas);

    Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "results comparison";

    for(unsigned int i = 0 ; i < nbMeas ; i++)
    {
        Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "measures[" << i << "]:";

        std::stringstream ss;
        ss << "meas_" << i;
        double trueXBeacon = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "xBeacon") );
        double trueYBeacon = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "yBeacon") );
        double trueRange   = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "range") );
        double trueTheta   = docMeas.getFloatData( docMeas.getChild( docMeas.getChild(docMeas.root(), ss.str()), "theta") );

        Circle target = measures[i].first;
        Eigen::Vector2d meas = measures[i].second;

        Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * xTarget=" << target.x() << "   vs trueXBeacon=" << trueXBeacon;
        Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * yTarget=" << target.y() << "   vs trueYBeacon=" << trueYBeacon;
        Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * range="   << meas(0)    << "   vs trueRange="   << trueRange;
        Log( DEBUG ) << "test_DetectedCircle_5" << " - " << "  * theta="   << meas(1)    << "   vs trueTheta="   << trueTheta;


        BOOST_CHECK_CLOSE( target.x(), trueXBeacon, 1.f);
        BOOST_CHECK_CLOSE( target.y(), trueYBeacon, 1.f);
        BOOST_CHECK( abs(meas(0) - trueRange) < 0.01 );
        BOOST_CHECK( abs(meas(1) - trueTheta) < 0.01);
    }
}


BOOST_AUTO_TEST_SUITE_END()
